#!/usr/bin/env python3
"""
Takes an existing bag and produces a new one with everything the old one
had PLUS /loadcell_data_filtered, computed by reading /loadcell_data
directly out of the bag file and running the same Kalman filter as
ForceFilterNode.py - entirely offline, no ROS graph, no DDS, no live replay.

This replaces the replay-through-a-live-node approach (augment_bag_with_filter.py)
for this exact purpose: that approach turned out to have a real duplicate-
delivery bug (ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET causing DDS to deliver
this machine's own traffic via multiple redundant paths, inflating message
counts ~4.5x) plus general orphaned-process/cleanup complexity. None of
that is possible here - this script never starts a ROS node, never
publishes/subscribes to anything, and just streams bytes from one bag file
to another with an in-memory filter computation in between. Every other
topic (video, actuator telemetry, etc.) is copied through byte-for-byte,
unmodified, at its original timestamp.

The Kalman recursion is the same scalar math as tune_filter_offline.py -
keep the two (and ForceFilterNode.py) in sync if the model ever changes.

Usage:
    python3 augment_bag_offline.py /path/to/old_bag /path/to/new_bag \\
        --measurement-noise-variance 3e6 --process-noise-accel-stddev 1000
"""

import argparse
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import WrenchStamped

SOURCE_TOPIC = '/loadcell_data'
FILTERED_TOPIC = '/loadcell_data_filtered'


def guess_storage_id(bag_path: str) -> str:
    # Matches this project's bags (all recorded with rosbag2's mcap plugin,
    # confirmed via `ros2 bag info` throughout this project) - .db3 bags
    # would need 'sqlite3' instead.
    return 'mcap'


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('old_bag', help='Path to the existing rosbag2 directory to read')
    parser.add_argument('new_bag', help='Output path for the new augmented bag (must not exist)')
    parser.add_argument('--measurement-noise-variance', type=float, default=3e6)
    parser.add_argument('--process-noise-accel-stddev', type=float, default=1000.0)
    args = parser.parse_args()

    if os.path.exists(args.new_bag):
        print(f'ERROR: {args.new_bag} already exists - delete it or pick a new path.')
        sys.exit(1)

    storage_id = guess_storage_id(args.old_bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.old_bag, storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.new_bag, storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )

    topics = reader.get_all_topics_and_types()
    max_id = max((t.id for t in topics), default=0)
    source_topic_meta = next((t for t in topics if t.name == SOURCE_TOPIC), None)
    if source_topic_meta is None:
        print(f'ERROR: {SOURCE_TOPIC} not found in {args.old_bag}.')
        sys.exit(1)

    # If the source bag already has a /loadcell_data_filtered (e.g. it was
    # recorded live through a force_filter node - which may itself have
    # been affected by DDS duplicate-delivery, or just superseded params),
    # drop it entirely rather than copying it through: the whole point of
    # this script is to produce a correct one, not stack a fresh copy on
    # top of a stale/possibly-corrupted one under the same topic name.
    if any(t.name == FILTERED_TOPIC for t in topics):
        print(f'NOTE: source bag already has {FILTERED_TOPIC} - dropping it '
              'and replacing with a freshly computed one.')
    topics = [t for t in topics if t.name != FILTERED_TOPIC]

    for t in topics:
        writer.create_topic(t)
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=max_id + 1,
        name=FILTERED_TOPIC,
        type=source_topic_meta.type,
        serialization_format=source_topic_meta.serialization_format,
        offered_qos_profiles=source_topic_meta.offered_qos_profiles,
    ))

    # Kalman state - same scalar formulation as tune_filter_offline.py.
    q = args.process_noise_accel_stddev ** 2
    R = args.measurement_noise_variance
    f = v = None
    p11 = p12 = p22 = 0.0
    last_stamp_ns = None

    total = 0
    filtered_count = 0
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        if topic == FILTERED_TOPIC:
            continue  # dropping the source's stale/possibly-corrupted copy
        writer.write(topic, data, recv_ns)
        total += 1

        if topic != SOURCE_TOPIC:
            continue

        msg = deserialize_message(data, WrenchStamped)
        z = msg.wrench.force.z
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if f is None:
            f, v = z, 0.0
            p11, p12, p22 = 1.0, 0.0, 1.0
        else:
            dt = (stamp_ns - last_stamp_ns) / 1e9
            if dt <= 0:
                dt = 1e-6

            # Predict
            f_pred = f + v * dt
            v_pred = v
            q11 = q * dt**3 / 3
            q12 = q * dt**2 / 2
            q22 = q * dt
            p11_pred = p11 + 2 * dt * p12 + dt**2 * p22 + q11
            p12_pred = p12 + dt * p22 + q12
            p22_pred = p22 + q22

            # Update (H = [1, 0])
            y = z - f_pred
            S = p11_pred + R
            k1 = p11_pred / S
            k2 = p12_pred / S

            f = f_pred + k1 * y
            v = v_pred + k2 * y
            p11 = p11_pred * (1 - k1)
            p12 = p12_pred * (1 - k1)
            p22 = p22_pred - k2 * p12_pred

        last_stamp_ns = stamp_ns

        out = WrenchStamped()
        out.header = msg.header
        out.wrench.force.z = f
        writer.write(FILTERED_TOPIC, serialize_message(out), recv_ns)
        filtered_count += 1

    print(f'Done. {total} messages copied, {filtered_count} filtered '
          f'{SOURCE_TOPIC} messages generated -> {args.new_bag}')


if __name__ == '__main__':
    main()
