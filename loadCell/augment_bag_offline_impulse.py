#!/usr/bin/env python3
"""
Takes an existing bag and produces a new one with everything the old one
had PLUS /loadcell_data_impulse, computed by reading
/loadcell_data_calibrated directly out of the bag file and running a
cumulative trapezoidal integral of force over time - entirely offline, no
ROS graph, no DDS, no live replay.

Same rationale as augment_bag_offline.py/augment_bag_offline_calibrate.py
(which this was copied from): no live node, no DDS duplicate-delivery risk,
just streams bytes from one bag file to another with an in-memory
computation in between. Every other topic (video, actuator telemetry, the
calibrated force channel itself, etc.) is copied through byte-for-byte,
unmodified, at its original timestamp.

Unlike calibration, this IS stateful (like the Kalman filter step) - each
message's impulse value is the running total of everything integrated so
far, not just a function of that one message. The very first sample on the
source topic has nothing to integrate against yet, so it's emitted as 0.
Trapezoidal integration: for consecutive samples (f0, t0) -> (f1, t1),
the contribution to the running total is 0.5 * (f0 + f1) * (t1 - t0).

Usage:
    python3 augment_bag_offline_impulse.py /path/to/old_bag /path/to/new_bag
"""

import argparse
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import WrenchStamped

SOURCE_TOPIC = '/loadcell_data_calibrated'
IMPULSE_TOPIC = '/loadcell_data_impulse'


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

    # If the source bag already has a /loadcell_data_impulse (e.g. produced
    # by an earlier run of this script), drop it entirely rather than
    # copying it through: the whole point of this script is to produce a
    # correct one, not stack a fresh copy on top of a stale one under the
    # same topic name.
    if any(t.name == IMPULSE_TOPIC for t in topics):
        print(f'NOTE: source bag already has {IMPULSE_TOPIC} - dropping it '
              'and replacing with a freshly computed one.')
    topics = [t for t in topics if t.name != IMPULSE_TOPIC]

    for t in topics:
        writer.create_topic(t)
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=max_id + 1,
        name=IMPULSE_TOPIC,
        type=source_topic_meta.type,
        serialization_format=source_topic_meta.serialization_format,
        offered_qos_profiles=source_topic_meta.offered_qos_profiles,
    ))

    # Running trapezoidal integral - same "carry state between messages"
    # shape as the Kalman filter step, just a much simpler update.
    cumulative_impulse = 0.0
    last_force = None
    last_stamp_ns = None

    total = 0
    impulse_count = 0
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        if topic == IMPULSE_TOPIC:
            continue  # dropping the source's stale copy
        writer.write(topic, data, recv_ns)
        total += 1

        if topic != SOURCE_TOPIC:
            continue

        msg = deserialize_message(data, WrenchStamped)
        z = msg.wrench.force.z
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if last_force is None:
            # Nothing to integrate against yet on the very first sample.
            pass
        else:
            dt = (stamp_ns - last_stamp_ns) / 1e9
            if dt > 0:
                cumulative_impulse += 0.5 * (last_force + z) * dt
            # else: out-of-order or duplicate timestamp - contributes
            # nothing rather than integrating over a bogus/negative dt.

        last_force = z
        last_stamp_ns = stamp_ns

        out = WrenchStamped()
        out.header = msg.header
        out.wrench.force.z = cumulative_impulse
        writer.write(IMPULSE_TOPIC, serialize_message(out), recv_ns)
        impulse_count += 1

    print(f'Done. {total} messages copied, {impulse_count} impulse '
          f'{SOURCE_TOPIC} messages generated -> {args.new_bag}')


if __name__ == '__main__':
    main()
