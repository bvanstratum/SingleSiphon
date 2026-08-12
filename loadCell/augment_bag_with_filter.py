#!/usr/bin/env python3
"""
Takes an existing bag (no /loadcell_data_filtered in it) and produces a new
bag with everything the old one had PLUS /loadcell_data_filtered, by
replaying the old bag through a live force_filter node and re-recording -
no need to physically re-run the experiment just to add the filtered topic.

Does NOT start foxglove_bridge (unlike replay_bag_through_filter.py) - this
is a batch/offline operation, nothing needs to be watched live. Open the
resulting bag as a local file in Foxglove afterward (header-stamp plotting
works correctly there, unlike over a live foxglove_bridge connection).

Usage:
    python3 augment_bag_with_filter.py /path/to/old_bag /path/to/new_bag \\
        --measurement-noise-variance 3e6 --process-noise-accel-stddev 1000

Runs replay at real-time (--rate 1.0) by default rather than sped up - the
recorder's subscriptions are BEST_EFFORT, same as the original topic, and
speeding up replay raises the risk of message drops under load, which would
make the new bag's filtered output subtly wrong. Slow down further
(--rate 0.5) if you suspect drops in the output.
"""

import argparse
import os
import signal
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
# eliminated it in a controlled A/B test. Everything this script does is
# single-machine anyway, so there's no reason to allow subnet-wide discovery
# here regardless of what the shell's own environment has set.
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOCALHOST_ONLY'] = '1'


def check_no_existing_force_filter():
    """Refuses to start if a force_filter is already running. Multiple
    instances alive at once will all publish onto /loadcell_data_filtered
    independently - the recorder then captures the union of all of them,
    inflating the message count and interleaving duplicate/out-of-order
    header stamps from the different publishers. This is a real bug we hit:
    leftover orphaned instances from an earlier aborted run silently
    corrupted a later recording this exact way."""
    result = subprocess.run(['pgrep', '-af', 'lib/singleSiphon/force_filter'],
                             capture_output=True, text=True)
    if result.stdout.strip():
        print('ERROR: force_filter is already running:\n')
        print(result.stdout)
        print('Kill it first (these would double-publish onto '
              '/loadcell_data_filtered and corrupt the output) - '
              "e.g. `pkill -INT -f force_filter`, then re-run.")
        sys.exit(1)


def stop_gracefully(proc, name, timeout=5.0):
    """rclpy nodes are built to shut down cleanly on SIGINT (like Ctrl+C),
    not SIGTERM. But signaling proc.pid alone isn't enough either: `ros2 run`
    (and `ros2 bag record`) don't exec into the actual node - they fork a
    child process for it - so proc.pid only reaches the CLI wrapper, and the
    real node survives as an orphan even after the wrapper dies. Confirmed
    live: killing the tracked PID left the actual force_filter process
    reparented to systemd, still running, still publishing.

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
    parser.add_argument('old_bag', help='Path to the existing rosbag2 directory to replay')
    parser.add_argument('new_bag', help='Output path for the new augmented bag (must not exist)')
    parser.add_argument('--measurement-noise-variance', type=float, default=3e6)
    parser.add_argument('--process-noise-accel-stddev', type=float, default=1000.0)
    parser.add_argument('--rate', type=float, default=1.0, help='Bag playback rate')
    args = parser.parse_args()

    check_no_existing_force_filter()

    singlesiphon_setup = os.path.expanduser(
        '~/SIPHION_Master_Folder/singleSiphon/install/setup.bash')
    force_filter_cmd = (
        f'source /opt/ros/jazzy/setup.bash && '
        f'source {singlesiphon_setup} && '
        f'exec ros2 run singleSiphon force_filter --ros-args '
        f'-p measurement_noise_variance:={args.measurement_noise_variance} '
        f'-p process_noise_accel_stddev:={args.process_noise_accel_stddev}'
    )
    print(f'Starting force_filter (measurement_noise_variance='
          f'{args.measurement_noise_variance:g}, process_noise_accel_stddev='
          f'{args.process_noise_accel_stddev:g})...')
    force_filter_proc = subprocess.Popen(['bash', '-c', force_filter_cmd], start_new_session=True)
    time.sleep(1.5)

    print(f'Starting recorder -> {args.new_bag}...')
    # Same exclude-regex as the live session's own rosbag_record_action -
    # skips redundant image transports the old bag also recorded, so this
    # doesn't just double the amount of camera data for no reason.
    record_proc = subprocess.Popen([
        'ros2', 'bag', 'record', '-a',
        '--exclude-regex', r'image_raw$|image_raw/(theora|zstd|compressedDepth)$',
        '-o', args.new_bag,
    ], start_new_session=True)
    # Recorder needs a moment to discover topics and establish subscriptions
    # via DDS before playback starts, or it can miss the first messages.
    time.sleep(2.0)

    # `ros2 bag record -o` exits immediately (code 1) if new_bag already
    # exists (e.g. re-running with the same output path as a previous test)
    # - without this check the script would blindly play the whole bag
    # through a recorder that isn't actually running, then report success
    # and point you at a stale file from whatever wrote there last.
    if record_proc.poll() is not None:
        print(f'ERROR: recorder exited immediately (code {record_proc.returncode}) - '
              f'"{args.new_bag}" most likely already exists. Delete it or pick a '
              'new output path, then re-run.')
        stop_gracefully(force_filter_proc, 'force_filter')
        sys.exit(1)

    print(f'Playing {args.old_bag} at rate {args.rate}...')
    try:
        subprocess.run(['ros2', 'bag', 'play', args.old_bag, '--rate', str(args.rate)])
    finally:
        print('Playback done - stopping recorder and force_filter...')
        stop_gracefully(record_proc, 'recorder')
        stop_gracefully(force_filter_proc, 'force_filter')

    print(f'Done. New bag written to {args.new_bag} - open it as a local '
          'file in Foxglove to review.')


if __name__ == '__main__':
    main()
