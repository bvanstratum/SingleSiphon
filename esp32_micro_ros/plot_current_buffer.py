#!/usr/bin/env python3
"""
Plot a saved current buffer .npz file.

Usage:
    python3 plot_current_buffer.py current_buffer_20260612_123456.npz
    python3 plot_current_buffer.py my_run.npz --volts                  # convert ADC counts to volts
    python3 plot_current_buffer.py my_run.npz --time-axis ros          # x-axis = ROS epoch seconds
    python3 plot_current_buffer.py my_run.npz --time-axis relative     # x-axis = seconds since buffer start (default)
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt

ADC_MAX  = 4095   # 12-bit
ADC_VREF = 3.3    # volts


def main():
    parser = argparse.ArgumentParser(description='Plot a saved current buffer .npz file.')
    parser.add_argument('path')
    parser.add_argument('--volts', action='store_true', help='convert ADC counts to volts')
    parser.add_argument('--time-axis', choices=['ros', 'relative'], default='relative',
                         help="'ros': absolute ROS epoch seconds (matches SerialDebug log timestamps). "
                              "'relative': seconds since buffer start (default).")
    args = parser.parse_args()

    npz             = np.load(args.path)
    samples         = npz['samples'].astype(np.float32)
    timestamps      = npz['timestamps']
    sample_rate_hz  = int(npz['sample_rate_hz'])

    if args.time_axis == 'ros':
        # Strip the constant integer prefix so tick labels are readable, but print
        # the base in the axis label so you can add it back to match log timestamps.
        # e.g. ticks show 897.5, 912.3 ... and label says "+ 1782849000 s"
        t_base = int(timestamps[0] / 100) * 100
        time   = timestamps - t_base
        xlabel = f'ROS time  +{t_base} (s)  [log timestamp = tick + {t_base}]'
    else:
        time    = timestamps - timestamps[0]
        xlabel  = 'Time since buffer start (s)'

    if args.volts:
        samples = samples / ADC_MAX * ADC_VREF
        ylabel  = 'Voltage (V)'
    else:
        ylabel = 'ADC counts (0–4095)'

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(time, samples, linewidth=0.5)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    duration_s = timestamps[-1] - timestamps[0]
    ax.set_title(f'{args.path}  —  {len(samples)} samples @ {sample_rate_hz} Hz  ({duration_s:.1f}s)')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
