#!/usr/bin/env python3
"""
Takes an existing bag and produces a new one with everything the old one
had PLUS /loadcell_data_calibrated, computed by reading
/loadcell_data_filtered directly out of the bag file and applying the same
linear calibration as ForceCalibratorNode.py - entirely offline, no ROS
graph, no DDS, no live replay.

Same rationale as augment_bag_offline.py (which this was copied from, for
the Kalman-filter step): no live node, no DDS duplicate-delivery risk, just
streams bytes from one bag file to another with an in-memory calibration
computation in between. Every other topic (video, actuator telemetry, the
filtered force channel itself, etc.) is copied through byte-for-byte,
unmodified, at its original timestamp.

Calibration here is stateless (unlike the Kalman filter) - each message's
calibrated value only depends on that message, not on any running state or
elapsed time. Keep the slope default in sync with ForceCalibratorNode.py if
the calibration is ever redone.

No intercept: dropped on the assumption that it reflected the original
calibration run's taring procedure rather than something intrinsic to the
sensor - see esp32LoadCell_mROS.ino's calibration comment for the full
reasoning.

Usage:
    python3 augment_bag_offline_calibrate.py /path/to/old_bag /path/to/new_bag \\
        --calibration-slope -0.08355
"""

import argparse
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import WrenchStamped

SOURCE_TOPIC = '/loadcell_data_filtered'
CALIBRATED_TOPIC = '/loadcell_data_calibrated'


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
    parser.add_argument('--calibration-slope', type=float, default=-0.08355)
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

    # If the source bag already has a /loadcell_data_calibrated (e.g. it was
    # recorded live through a force_calibrator node, or produced by an
    # earlier run of this script with different coefficients), drop it
    # entirely rather than copying it through: the whole point of this
    # script is to produce a correct one, not stack a fresh copy on top of
    # a stale one under the same topic name.
    if any(t.name == CALIBRATED_TOPIC for t in topics):
        print(f'NOTE: source bag already has {CALIBRATED_TOPIC} - dropping it '
              'and replacing with a freshly computed one.')
    topics = [t for t in topics if t.name != CALIBRATED_TOPIC]

    for t in topics:
        writer.create_topic(t)
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=max_id + 1,
        name=CALIBRATED_TOPIC,
        type=source_topic_meta.type,
        serialization_format=source_topic_meta.serialization_format,
        offered_qos_profiles=source_topic_meta.offered_qos_profiles,
    ))

    total = 0
    calibrated_count = 0
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        if topic == CALIBRATED_TOPIC:
            continue  # dropping the source's stale copy
        writer.write(topic, data, recv_ns)
        total += 1

        if topic != SOURCE_TOPIC:
            continue

        msg = deserialize_message(data, WrenchStamped)
        z = msg.wrench.force.z
        calibrated = args.calibration_slope * z

        out = WrenchStamped()
        out.header = msg.header
        out.wrench.force.z = calibrated
        writer.write(CALIBRATED_TOPIC, serialize_message(out), recv_ns)
        calibrated_count += 1

    print(f'Done. {total} messages copied, {calibrated_count} calibrated '
          f'{SOURCE_TOPIC} messages generated -> {args.new_bag}')


if __name__ == '__main__':
    main()
