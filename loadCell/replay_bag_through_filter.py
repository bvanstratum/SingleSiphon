#!/usr/bin/env python3
"""
Replays an old rosbag through a live force_filter node so you can evaluate
Kalman parameters against real past data in Foxglove, without needing the
full frequencyControlDemo_withVideo.py launch (cameras, actuators, etc).

Starts (if not already running):
  - foxglove_bridge, so Foxglove can connect
  - force_filter, with the noise parameters given on the command line

Then plays the bag in the foreground - /loadcell_data (from the bag) and
/loadcell_data_filtered (from force_filter) both appear live for Foxglove
to subscribe to and plot side by side. Ctrl+C stops playback and tears
down anything this script started.

Usage:
    python3 replay_bag_through_filter.py /path/to/old/bag \\
        --measurement-noise-variance 3.2e6 \\
        --process-noise-accel-stddev 100.0

    # Sweep a different process noise without touching the launch file:
    python3 replay_bag_through_filter.py /path/to/old/bag --process-noise-accel-stddev 500
"""

import argparse
import os
import signal
import socket
import subprocess
import sys
import time

# Confirmed live: with ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET (a real default
# on this machine - it has a /15 WiFi subnet), DDS discovers/matches over
# the network in addition to localhost, and this machine's own traffic gets
# delivered via multiple redundant paths (loopback + the WiFi interface),
# causing genuine transport-level duplicate delivery - invisible to
# `ros2 topic info` (still shows exactly 1 publisher/1 subscriber), but
# produced a steady ~4.5x inflation of /loadcell_data_filtered's message
# count for an entire recording session. Forcing localhost-only discovery
# eliminated it in a controlled A/B test. This only affects ROS/DDS traffic
# between the nodes this script starts - foxglove_bridge's own websocket
# server (separate from DDS) is unaffected, so remote Foxglove clients can
# still connect normally.
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOCALHOST_ONLY'] = '1'


def is_port_open(port: int, host: str = '127.0.0.1') -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def check_no_existing_force_filter():
    """Refuses to start if a force_filter is already running. Multiple
    instances alive at once will all publish onto /loadcell_data_filtered
    independently - a downstream recorder (or Foxglove) then sees the union
    of all of them, inflating message counts and interleaving duplicate/
    out-of-order header stamps from the different publishers. This is a
    real bug we hit: leftover orphaned instances from an earlier aborted
    run silently corrupted a later recording this exact way."""
    result = subprocess.run(['pgrep', '-af', 'lib/singleSiphon/force_filter'],
                             capture_output=True, text=True)
    if result.stdout.strip():
        print('ERROR: force_filter is already running:\n')
        print(result.stdout)
        print('Kill it first (these would double-publish onto '
              '/loadcell_data_filtered) - e.g. `pkill -INT -f force_filter`, '
              'then re-run.')
        sys.exit(1)


def stop_gracefully(proc, name, timeout=5.0):
    """ROS2 nodes are built to shut down cleanly on SIGINT (like Ctrl+C).
    But signaling proc.pid alone isn't enough either: `ros2 run` doesn't
    exec into the actual node - it forks a child process for it - so
    proc.pid only reaches the CLI wrapper, and the real node survives as an
    orphan even after the wrapper dies. Confirmed live: killing the tracked
    PID left the actual force_filter process reparented to systemd, still
    running, still publishing.

    Fixed by launching with start_new_session=True (so the wrapper and
    whatever it forks all share one process group) and signaling that whole
    group with os.killpg instead of the single tracked PID. SIGINT first,
    SIGKILL fallback so this can't hang either."""
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return  # already gone

    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f'{name} did not exit on SIGINT within {timeout}s - killing its process group')
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag_path', help='Path to the old rosbag2 directory to replay')
    parser.add_argument('--measurement-noise-variance', type=float, default=3e6)
    parser.add_argument('--process-noise-accel-stddev', type=float, default=1000.0)
    parser.add_argument('--rate', type=float, default=1.0, help='Bag playback rate')
    args = parser.parse_args()

    check_no_existing_force_filter()

    started = []  # list of (name, proc)

    if is_port_open(8765):
        print('foxglove_bridge already running on 8765 - reusing it.')
    else:
        print('Starting foxglove_bridge...')
        started.append(('foxglove_bridge',
                         subprocess.Popen(['ros2', 'run', 'foxglove_bridge', 'foxglove_bridge'],
                                           start_new_session=True)))
        time.sleep(1.0)

    print(f'Starting force_filter (measurement_noise_variance='
          f'{args.measurement_noise_variance:g}, process_noise_accel_stddev='
          f'{args.process_noise_accel_stddev:g})...')
    # force_filter lives in the singleSiphon workspace, not a system ROS
    # package - "ros2 run" silently fails with "No executable found" if that
    # workspace's install/setup.bash isn't already sourced in the calling
    # shell. Source it here explicitly instead of relying on the caller to
    # remember, since a failed start here is easy to miss (no filtered
    # topic appears, but everything else - bag playback, foxglove_bridge -
    # looks fine).
    singlesiphon_setup = os.path.expanduser(
        '~/SIPHION_Master_Folder/singleSiphon/install/setup.bash')
    force_filter_cmd = (
        f'source /opt/ros/jazzy/setup.bash && '
        f'source {singlesiphon_setup} && '
        f'exec ros2 run singleSiphon force_filter --ros-args '
        f'-p measurement_noise_variance:={args.measurement_noise_variance} '
        f'-p process_noise_accel_stddev:={args.process_noise_accel_stddev}'
    )
    started.append(('force_filter', subprocess.Popen(['bash', '-c', force_filter_cmd],
                                                       start_new_session=True)))
    time.sleep(1.0)

    print(f'Playing {args.bag_path} at rate {args.rate}...')
    print('Subscribe to /loadcell_data and /loadcell_data_filtered in Foxglove now.')
    try:
        subprocess.run(['ros2', 'bag', 'play', args.bag_path, '--rate', str(args.rate)])
    except KeyboardInterrupt:
        pass
    finally:
        for name, p in started:
            stop_gracefully(p, name)


if __name__ == '__main__':
    main()
