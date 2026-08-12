#!/usr/bin/python3.12
# Must run under the ROS Python 3.12, not miniconda python3 (3.13).
# rosbag2_py .so files are compiled for Python 3.12.
# Run directly:   /usr/bin/python3.12 npz_to_bag.py <file.npz>
"""
Convert a telemetry .npz (from save_current_buffer_sync.py) to a rosbag2 (mcap)
so all three channels can be loaded alongside the main recording bag in PlotJuggler
on the same time axis.

Telemetry .npz keys (new format):
  current      uint16  raw ADC counts
  actual_pos   float32 shaft position (millidegrees)
  desired_pos  float32 desired shaft position (millidegrees)
  timestamps   float64 ROS epoch seconds
  sample_rate_hz, ros_anchor_ns, ticks_to_mdeg

Topics written to the bag:
  actuator_1/current_hires     Float32  ADC counts
  actuator_1/actual_pos_hires  Float32  millidegrees
  actuator_1/desired_pos_hires Float32  millidegrees

In PlotJuggler: open the main recording bag, then File -> Open -> add this bag.
Both appear on the same timeline because timestamps are ROS-epoch-aligned.

Usage:
    /usr/bin/python3.12 npz_to_bag.py telemetry_buffer_20260707_121309.npz
    /usr/bin/python3.12 npz_to_bag.py telemetry_buffer_20260707_121309.npz --out ~/SIPHION_Master_Folder/esp32_bags/run1/
"""

import argparse
import numpy as np
import os
import shutil

import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import Float32


def _make_writer(out_path: str) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=out_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return writer


def _add_topic(writer: rosbag2_py.SequentialWriter, topic_id: int, name: str):
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=topic_id,
        name=name,
        type='std_msgs/msg/Float32',
        serialization_format='cdr',
    ))


def npz_to_bag(npz_path: str, out_path: str | None = None) -> str:
    """Convert a telemetry .npz into a standalone rosbag2 (mcap).

    Writes three topics: current_hires, actual_pos_hires, desired_pos_hires.
    Returns the output bag directory path.
    """
    npz = np.load(npz_path)
    timestamps  = npz['timestamps']           # float64 ROS epoch seconds
    rate_hz     = int(npz['sample_rate_hz'])

    # Support both old single-channel format (samples) and new telemetry format
    if 'current' in npz:
        current     = npz['current'].astype(np.float32)
        actual_pos  = npz['actual_pos'].astype(np.float32)
        desired_pos = npz['desired_pos'].astype(np.float32)
        channels = [
            ('actuator_1/current_hires',     current),
            ('actuator_1/actual_pos_hires',  actual_pos),
            ('actuator_1/desired_pos_hires', desired_pos),
        ]
    else:
        # Legacy single-channel format
        channels = [('actuator_1/current_hires', npz['samples'].astype(np.float32))]

    out_path = out_path or os.path.splitext(npz_path)[0] + '_bag'
    out_path = os.path.expanduser(out_path)

    if os.path.exists(out_path):
        shutil.rmtree(out_path)

    writer = _make_writer(out_path)
    for i, (name, _) in enumerate(channels):
        _add_topic(writer, i, name)

    msg = Float32()
    for i, t in enumerate(timestamps):
        t_ns = int(t * 1e9)
        for name, data in channels:
            msg.data = float(data[i])
            writer.write(name, serialize_message(msg), t_ns)

    del writer  # flushes and closes

    duration = timestamps[-1] - timestamps[0]
    n = len(timestamps)
    print(f'Wrote {n} samples ({duration:.1f}s @ {rate_hz}Hz) -> {out_path}')
    for name, _ in channels:
        print(f'  topic: {name}')
    print(f'Time range: {timestamps[0]:.3f} -> {timestamps[-1]:.3f}')
    return out_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('npz', help='Path to .npz from save_current_buffer_sync.py')
    parser.add_argument('--out', default=None,
                        help='Output bag directory (default: <npz_name>_bag/ next to the npz)')
    args = parser.parse_args()
    npz_to_bag(args.npz, out_path=args.out)


if __name__ == '__main__':
    main()
