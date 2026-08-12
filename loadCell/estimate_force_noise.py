#!/usr/bin/env python3
"""
Estimates ForceFilterNode's Kalman noise parameters from real loadcell data.

measurement_noise_variance is estimated directly: point the load cell at a
segment where the true force wasn't actually changing (cell sitting
unloaded, or under a fixed static weight) and the sample variance of
wrench.force.z *is* the measurement noise - any spread in a truly-static
signal is by definition sensor noise, not signal.

process_noise_accel_stddev is NOT estimated here. It answers "how fast can
real force plausibly accelerate", which isn't recoverable from a static
recording (a static segment has no real acceleration to measure). Tune it
by eye instead: run the filter against a bag with real dynamic force
changes (a real actuator run) and increase it until the filtered trace
stops lagging the true motion, decrease it until it stops looking noisy.
This script only reports the raw signal's own timing/dynamics as context
for that by-eye tuning.

Usage:
    # Analyze a recorded bag (point it at a segment you know was static)
    python3 estimate_force_noise.py /path/to/bag_dir

    # Or capture live for N seconds (default 10) - stand the setup down /
    # leave the load cell unloaded and undisturbed while it runs
    python3 estimate_force_noise.py --live [--duration 10]
 
    
    this was my usage
    ase) brian@brian-Yoga-Slim-7-linux:~/SIPHION_Master_Folder$ /usr/bin/python3 loadCell/estimate_force_noise.py ./esp32_bags/20260722_13475
4/computer.mcap --start-sec 5.0 --end-sec 25.0
Sliced to [5.0, 25.0) sec: 380262 -> 39551 samples

Samples:              39551
Duration:              20.00 s
Mean sample interval:  0.51 ms (min 0.22 / max 19.82)
Mean force:            629.0204
Std dev:               1794.1577

measurement_noise_variance = 3.219e+06
(only valid if this segment was truly static - a nonzero trend or real force changes in the window will inflate this number)


"""

import argparse
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped

TOPIC = '/loadcell_data'


def read_bag(bag_path: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    samples = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, WrenchStamped)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        samples.append((stamp_ns, msg.wrench.force.z))
    return samples


def read_live(duration: float):
    rclpy.init()
    node = Node('force_noise_estimator')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)
    samples = []

    def on_msg(msg: WrenchStamped):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        samples.append((stamp_ns, msg.wrench.force.z))

    node.create_subscription(WrenchStamped, TOPIC, on_msg, qos)
    print(f'Listening on {TOPIC} for {duration:.1f}s - keep the load cell '
          'still and undisturbed...')
    end_time = node.get_clock().now().nanoseconds + int(duration * 1e9)
    try:
        while node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return samples


def slice_by_time(samples, start_sec, end_sec):
    """Keeps only samples within [start_sec, end_sec) of the first sample's
    timestamp - i.e. the same "elapsed seconds since bag start" number shown
    on Foxglove's playback scrubber. There's no meaningful integer index into
    a bag (it's just a time-ordered message stream, not a fixed-size array),
    so time is the only handle that lines up with what you actually saw
    while scrubbing to find the static segment."""
    if not samples:
        return samples
    t0 = samples[0][0]
    lo = t0 + int(start_sec * 1e9) if start_sec is not None else t0
    hi = t0 + int(end_sec * 1e9) if end_sec is not None else float('inf')
    return [(t, f) for t, f in samples if lo <= t < hi]


def analyze(samples):
    if len(samples) < 2:
        print(f'Only got {len(samples)} sample(s) - need at least a few '
              'seconds of data. Nothing to compute.')
        sys.exit(1)

    forces = [f for _, f in samples]
    stamps = [t for t, _ in samples]
    dts = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]

    variance = statistics.variance(forces)
    mean = statistics.mean(forces)

    print(f'\nSamples:              {len(samples)}')
    print(f'Duration:              {(stamps[-1] - stamps[0]) / 1e9:.2f} s')
    print(f'Mean sample interval:  {statistics.mean(dts) * 1000:.2f} ms '
          f'(min {min(dts)*1000:.2f} / max {max(dts)*1000:.2f})')
    print(f'Mean force:            {mean:.4f}')
    print(f'Std dev:               {variance ** 0.5:.4f}')
    print(f'\nmeasurement_noise_variance = {variance:.6g}')
    print('(only valid if this segment was truly static - a nonzero trend '
          'or real force changes in the window will inflate this number)')

    # Linear-detrend sanity check: if a straight line explains a lot of the
    # spread, this segment probably wasn't actually static.
    n = len(forces)
    t = [(s - stamps[0]) / 1e9 for s in stamps]
    t_mean = statistics.mean(t)
    slope_num = sum((ti - t_mean) * (fi - mean) for ti, fi in zip(t, forces))
    slope_den = sum((ti - t_mean) ** 2 for ti in t)
    slope = slope_num / slope_den if slope_den else 0.0
    intercept = mean - slope * t_mean
    residuals = [fi - (slope * ti + intercept) for ti, fi in zip(t, forces)]
    residual_variance = statistics.variance(residuals)
    if variance > 0 and residual_variance < 0.8 * variance:
        print(f'\nWARNING: detrended variance ({residual_variance:.6g}) is '
              f'notably lower than raw variance ({variance:.6g}) - this '
              f'segment looks like it has a real trend (slope {slope:.4g} '
              'per second), not just noise. Consider a flatter segment or '
              'using the detrended value instead.')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag_path', nargs='?', help='Path to a rosbag2 directory')
    parser.add_argument('--live', action='store_true', help='Capture live instead of reading a bag')
    parser.add_argument('--duration', type=float, default=10.0, help='Live capture duration in seconds')
    parser.add_argument('--start-sec', type=float, default=None,
                         help='Only use samples from this many seconds after the bag start onward '
                              '(read this off Foxglove\'s playback scrubber)')
    parser.add_argument('--end-sec', type=float, default=None,
                         help='Only use samples up to this many seconds after the bag start '
                              '(e.g. the moment you started the real experiment)')
    args = parser.parse_args()

    if args.live:
        samples = read_live(args.duration)
    elif args.bag_path:
        samples = read_bag(args.bag_path)
    else:
        parser.error('Provide a bag_path or use --live')
        return

    if args.start_sec is not None or args.end_sec is not None:
        before = len(samples)
        samples = slice_by_time(samples, args.start_sec, args.end_sec)
        print(f'Sliced to [{args.start_sec}, {args.end_sec}) sec: '
              f'{before} -> {len(samples)} samples')

    analyze(samples)


if __name__ == '__main__':
    main()
