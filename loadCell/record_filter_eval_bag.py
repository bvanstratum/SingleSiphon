#!/usr/bin/env python3
"""
Records a small bag of just /loadcell_data and /loadcell_data_filtered, for
evaluating ForceFilterNode's Kalman output against the raw signal (load both
topics in the same Foxglove plot panel to compare directly).

Run this while frequencyControlDemo_withVideo.py (or anything else running
force_filter) is already up, then do whatever real force-changing motion
you want to evaluate the filter against.

Usage:
    python3 record_filter_eval_bag.py my_eval_run
    (Ctrl+C to stop recording)
"""

import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag_path', help='Output bag directory (must not already exist)')
    args = parser.parse_args()

    cmd = ['ros2', 'bag', 'record', '-o', args.bag_path,
           '/loadcell_data', '/loadcell_data_filtered']
    print(f'Running: {" ".join(cmd)}')
    print('Ctrl+C to stop recording.')
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
