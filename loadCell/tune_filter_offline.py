#!/usr/bin/env python3
"""
Offline, no-ROS Kalman filter tuning. Loads a bag ONCE into memory, then lets
you re-run the filter with different parameters and re-plot instantly - no
replaying through ROS/foxglove_bridge/force_filter per iteration.

Run with the interactive flag so the loaded data and functions stay in your
workspace between calls, MATLAB-script-style:

    /usr/bin/python3 -i tune_filter_offline.py /path/to/bag

Then at the interactive prompt it drops you into:

    >>> plot(measurement_noise_variance=3.2e6, process_noise_accel_stddev=100)
    >>> plot(process_noise_accel_stddev=500)   # instant re-plot, no reload
    >>> plot(process_noise_accel_stddev=2000, start_sec=5, end_sec=25)

Must run under /usr/bin/python3, not conda's python3 - rosbag2_py/rclpy are
only importable from the system Python (same conda-vs-system conflict as
everywhere else in this project).

The Kalman recursion in run_filter() is a plain-numpy copy of
ForceFilterNode.py's on_measurement logic, operating on a pre-loaded array
instead of a live subscription callback - keep the two in sync if the model
there changes.
<2026 7 24> i found that I like 3e6 and 1000 here but this looks different than in the foxglove 
"""

import sys

import numpy as np
import matplotlib.pyplot as plt

TOPIC = '/loadcell_data'


def load_bag(bag_path: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import WrenchStamped

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    stamps, forces = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, WrenchStamped)
        stamps.append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        forces.append(msg.wrench.force.z)

    t = (np.array(stamps) - stamps[0]) / 1e9
    z = np.array(forces)
    print(f'Loaded {len(z)} samples spanning {t[-1]:.2f}s from {bag_path}')
    return t, z


def run_filter(t, z, measurement_noise_variance, process_noise_accel_stddev):
    n = len(z)
    filtered = np.empty(n)
    x = np.array([z[0], 0.0])
    P = np.eye(2)
    filtered[0] = x[0]

    q_accel = process_noise_accel_stddev ** 2
    R = measurement_noise_variance
    H = np.array([[1.0, 0.0]])

    for i in range(1, n):
        dt = t[i] - t[i - 1]
        if dt <= 0:
            filtered[i] = x[0]
            continue

        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = q_accel * np.array([[dt**3 / 3, dt**2 / 2],
                                 [dt**2 / 2, dt]])

        x_pred = F @ x
        P_pred = F @ P @ F.T + Q

        y = z[i] - (H @ x_pred)[0]
        S = (H @ P_pred @ H.T)[0, 0] + R
        K = (P_pred @ H.T) / S
        x = x_pred + (K.flatten() * y)
        P = P_pred - K @ H @ P_pred

        filtered[i] = x[0]

    return filtered


def plot(measurement_noise_variance=3.219e6, process_noise_accel_stddev=10.0,
         start_sec=None, end_sec=None):
    mask = np.ones_like(t, dtype=bool)
    if start_sec is not None:
        mask &= t >= start_sec
    if end_sec is not None:
        mask &= t < end_sec
    tt, zz = t[mask], z[mask]

    filtered = run_filter(tt, zz, measurement_noise_variance, process_noise_accel_stddev)

    plt.figure()
    plt.plot(tt, zz, alpha=0.4, label='raw')
    plt.plot(tt, filtered, label='filtered')
    plt.xlabel('time (s)')
    plt.ylabel('force')
    plt.title(f'R={measurement_noise_variance:.3g}  '
              f'process_accel_std={process_noise_accel_stddev:.3g}')
    plt.legend()
    plt.show()
    return filtered


def moving_average(t, z, window_sec):
    """Centered moving average over a time window (not a sample count,
    since the real data isn't perfectly uniformly sampled). Near the start/
    end of the array there aren't enough neighbors for a full window, so it
    just averages over however many samples actually fall within
    [t[i]-window/2, t[i]+window/2] - the window shrinks at the edges rather
    than padding with zeros or wrapping around.

    Vectorized via searchsorted + cumsum instead of a per-point loop, so
    this stays fast (sub-second) even at hundreds of thousands of samples."""
    half = window_sec / 2.0
    lo_idx = np.searchsorted(t, t - half, side='left')
    hi_idx = np.searchsorted(t, t + half, side='right')
    cumsum = np.concatenate(([0.0], np.cumsum(z)))
    counts = hi_idx - lo_idx
    return (cumsum[hi_idx] - cumsum[lo_idx]) / counts


def plot_ma(window_sec=0.1, start_sec=None, end_sec=None):
    mask = np.ones_like(t, dtype=bool)
    if start_sec is not None:
        mask &= t >= start_sec
    if end_sec is not None:
        mask &= t < end_sec
    tt, zz = t[mask], z[mask]

    filtered = moving_average(tt, zz, window_sec)

    plt.figure()
    plt.plot(tt, zz, alpha=0.4, label='raw')
    plt.plot(tt, filtered, label='moving average')
    plt.xlabel('time (s)')
    plt.ylabel('force')
    plt.title(f'centered moving average, window={window_sec:.3g}s')
    plt.legend()
    plt.show()
    return filtered


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: /usr/bin/python3 -i tune_filter_offline.py /path/to/bag')
        sys.exit(1)
    t, z = load_bag(sys.argv[1])
    print('Loaded. Try:\n'
          '  plot(measurement_noise_variance=3.2e6, process_noise_accel_stddev=100)\n'
          '  plot_ma(window_sec=0.1)')
