#!/usr/bin/env python3
"""
Drives a randomized, replicated frequency-sweep experiment for the siphon
actuator and logs per-trial average force/power into a results table.

Protocol per trial (matches the conceptual design worked out for this):
  1. Operator-gated settle: blocks on terminal input() until the operator
     judges (via Foxglove, watching /loadcell_data_calibrated) that tank
     sloshing from the previous trial has died down enough to proceed.
  2. Tare loop: publishes a tare trigger to the loadcell board, then asks
     the operator (terminal input(), watching Foxglove) whether the
     resulting zero looks good. Re-tares until approved.
  3. Pulse: publishes the trial's target frequency to actuator_1/freq (no
     firmware trial_summary exists to wait on - see auto_pulse_summary()).
     transient_reject_pulses run first and are excluded entirely (letting
     the tank/motor settle into steady oscillation); f_bar/p_bar are then
     computed from a start/end snapshot delta across the following
     measurement_pulses only, both counts converted to durations via
     period = 1/freq_hz. f_bar comes from loadcell_data_impulse, p_bar from
     the micro_ros/energetics cumulativeEnergy_J delta (that total does NOT
     reset on tare, only on a firmware mode switch). actuator_1/freq is
     re-zeroed once the measurement window ends, so every trial starts
     pulsing from a clean stop.

Frequency order is fully randomized across all (frequency, replicate)
combinations, not blocked by frequency - so time-dependent drift (motor
heating, tare drift, etc.) doesn't get confounded with frequency.

Topic notes:
  - loadcell_tare_command is std_msgs/Bool (True = tare now).
  - loadcell_data_impulse and micro_ros/energetics are both real
    (esp32LoadCell_mROS.ino / esp32_micro_ros.ino). There never was an
    actuator_1/trial_summary publisher on the firmware side - this node used
    to assume one would eventually exist and block waiting for it
    (wait_for_summary() would hang forever); it now computes f_bar/p_bar
    itself instead, see auto_pulse_summary().
  - All topics use BEST_EFFORT QoS, same as every other topic in this
    package - confirmed against the actual firmware that RELIABLE QoS
    crashes these boards, so there's no choice here despite these being
    exactly the kind of one-shot control/result messages that would
    otherwise want reliable delivery. Real consequence: a dropped tare
    command just means the operator sees an unchanged reading and
    re-triggers it (the approval loop already tolerates that); a dropped
    loadcell_data_impulse/micro_ros/energetics sample just means f_bar/p_bar
    are computed from a slightly stale last-known value, not a hang.

Results are saved as a CSV via pandas once the full schedule completes.

Usage examples (parameters are all set via -p, see declare_parameter calls
below for the full list and their defaults):
  # Real run with bag recording on (one .mcap-containing directory per
  # accepted trial, under bags_dir):
  ros2 run singleSiphon test_runner --ros-args -p record_bags:=true

  # Override the frequency sweep and replicate count:
  ros2 run singleSiphon test_runner --ros-args \
    -p frequencies_hz:="[0.2, 0.4, 0.6, 0.8, 1.0]" -p replicates:=3

  # Dry run with no hardware connected at all (fake_summary() - control
  # flow only, no real ROS data):
  ros2 run singleSiphon test_runner --ros-args -p use_fake_summary:=true

  # Real loadcell/tare/impulse path, hand-perturbed instead of an automatic
  # pulse (manual_impulse_summary() - see its own docstring):
  ros2 run singleSiphon test_runner --ros-args -p use_manual_impulse_summary:=true

  # Warm start: resume an interrupted run (e.g. after a computer restart)
  # from where it left off. Point resume_from at the ORIGINAL run's
  # results_csv_path (its .pkl also works) - the node reloads that run's
  # shuffled schedule from the companion <results>_schedule.json written
  # alongside it, keeps writing into the same results/pickle files, and
  # skips straight to the first not-yet-completed trial with no re-prompt
  # for already-accepted trials. frequencies_hz/replicates are ignored
  # (the schedule file already encodes them):
  ros2 run singleSiphon test_runner --ros-args \
    -p resume_from:=~/SIPHION_Master_Folder/test_runner_results/results_20260812_140501.csv
"""

import json
import math
import os
import random
import shutil
import signal
import subprocess
import time
from datetime import datetime

# use_fake_summary (see declare_parameter below) skips running a real trial
# pulse and generates a random summary locally instead, so the
# trial-scheduling/logging loop can be exercised without any hardware
# connected - it bypasses message types/QoS/topic names and auto_pulse_summary()
# entirely, it only exercises this node's own control flow.

import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3, WrenchStamped

# Matches ActuatorTeleop's best_effort_qos in TeleopNode.py - actuator_1/freq
# and actuator_1/mode already have a BEST_EFFORT subscriber on the ESP32
# side, and a BEST_EFFORT publisher can't satisfy a RELIABLE subscriber's
# request, so this has to match rather than just defaulting to "safer."
# Real consequence: a dropped freq-command here means a trial silently
# never starts and wait_for_summary() blocks forever - watch Foxglove for
# the pulse actually beginning, not just this node's own log line.
best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)

MODE_FREQ = 0.0      # matches ActuatorTeleop.MODE_FREQ / ESP32 firmware
MODE_POSITION = 1.0  # matches ESP32 firmware's mode_callback(): (in->data >= 1.0f) ? POSITION : FREQ_SWEEP

# Same exclude pattern as the main system launch file's rosbag_record_action -
# skips the raw/theora/zstd duplicate camera transports, keeping only the
# compressed one.
BAG_EXCLUDE_REGEX = r'image_raw$|image_raw/(theora|zstd|compressedDepth)$'


def build_schedule(frequencies, replicates):
    schedule = [(f, r) for f in frequencies for r in range(replicates)]
    random.shuffle(schedule)
    return schedule


def schedule_path_for(results_csv_path):
    # Companion file holding the exact (already-shuffled) trial order for a
    # given results_csv_path, written unconditionally at the start of every
    # run() - so a resume_from later doesn't have to re-derive random order
    # from completed rows alone (which is ambiguous/impossible in general).
    return results_csv_path.replace('.csv', '_schedule.json')


class TestRunnerNode(Node):
    def __init__(self):
        super().__init__('test_runner')

        # self.declare_parameter('frequencies_hz', [0.2, 0.4, 0.6, 0.8, 1.0])
        self.declare_parameter('frequencies_hz', [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0])
        self.declare_parameter('replicates', 3)
        self.declare_parameter('use_fake_summary', False)
        self.declare_parameter('record_bags', False)
        # See manual_impulse_summary(): countdown-then-read-real-impulse test
        # mode, for exercising the real loadcell/tare/impulse path by hand
        # instead of running an automatic pulse via auto_pulse_summary().
        self.declare_parameter('use_manual_impulse_summary', False)
        self.declare_parameter('manual_test_duration_s', 10.0)
        # Real (non-fake, non-manual) trial timing, in PULSES not seconds -
        # converted to durations via period = 1/freq_hz inside
        # auto_pulse_summary(), so the same pulse counts apply at every
        # frequency instead of a fixed duration under/over-covering fast/slow
        # sweeps. transient_reject_pulses run first and are excluded from
        # f_bar/p_bar entirely (letting the tank/motor settle into steady
        # oscillation); only measurement_pulses' worth after that count
        # toward the average. Matches the original discard-3/average-5 design.
        self.declare_parameter('transient_reject_pulses', 3)
        self.declare_parameter('measurement_pulses', 5)
        # Dwell between the end-of-trial POSITION/FREQ_SWEEP mode bounce (see
        # run()) - long enough for the position controller to actually
        # settle at zero (bleeding off stiction current) before switching
        # back, not just for the mode_callback() reset itself (instantaneous).
        self.declare_parameter('mode_reset_settle_s', 0.5)
        self.declare_parameter(
            'results_csv_path',
            os.path.expanduser(
                '~/SIPHION_Master_Folder/test_runner_results/'
                f'results_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
            ),
        )
        self.declare_parameter(
            'bags_dir',
            os.path.expanduser('~/SIPHION_Master_Folder/test_runner_results/bags'),
        )
        # Warm-start: point this at a previous run's results_csv_path (or its
        # .pkl) to resume that exact run - same shuffled schedule (loaded from
        # the companion _schedule.json written alongside it), same output
        # files (appended to in place, not forked into a new timestamped
        # file), continuing right after the last completed trial. frequencies_hz/
        # replicates are ignored in this case since the schedule file already
        # encodes them. See run() for the resume logic.
        self.declare_parameter('resume_from', '')

        self.pub_freq = self.create_publisher(Float32, 'actuator_1/freq', best_effort_qos)
        self.pub_mode = self.create_publisher(Float32, 'actuator_1/mode', best_effort_qos)
        self.pub_tare = self.create_publisher(Bool, 'loadcell_tare_command', best_effort_qos)
        self.sub_impulse = self.create_subscription(
            WrenchStamped, 'loadcell_data_impulse', self.on_impulse, best_effort_qos)
        self.sub_energetics = self.create_subscription(
            WrenchStamped, 'micro_ros/energetics', self.on_energetics, best_effort_qos)

        self.latest_impulse = None
        # wrench.torque.x/y = cumulativeEnergy_J/cumulativeCharge_mAh - see
        # esp32_micro_ros.ino's field-mapping table. These are running
        # totals that only reset on a firmware mode switch (mode_callback()),
        # which this node never triggers per-trial, so per-trial "spent"
        # numbers have to come from a start/end snapshot delta - see
        # energetics_totals()/spin_briefly() and their use in run().
        self.latest_energetics = None
        # Raw values behind manual_impulse_summary()'s avg_force_mN, kept
        # around so run() can log them as their own columns alongside the
        # derived f_bar - not carried on the Vector3 summary object itself
        # since that interface is shared with fake_summary()/auto_pulse_summary(),
        # neither of which have such raw values to give.
        self.last_manual_impulse_mNs = None
        self.last_manual_elapsed_s = None
        # energy_spent_J/charge_spent_mAh for the CSV should cover exactly
        # the same measurement_pulses window p_bar is computed over (see
        # auto_pulse_summary()), not the whole trial including the
        # transient-reject phase - these hold that window's delta, reset to
        # None at the top of every wait_for_summary() call so a stale value
        # can't leak from an auto trial into a fake/manual one's row.
        self.last_measurement_energy_J = None
        self.last_measurement_charge_mAh = None
        # header.stamp (firmware's own clock, not this node's receive time)
        # of the micro_ros/energetics messages the above delta was computed
        # from - lets a rosbag/Foxglove timeline be indexed to exactly the
        # measurement window a trial's row covers. Same reset rule as above.
        self.last_measurement_t_start_sec = None
        self.last_measurement_t_end_sec = None
        # False if cumulativeEnergy_J/cumulativeCharge_mAh were ever seen
        # going backwards during the measurement window (only possible via a
        # firmware mode-switch reset or a reboot/reconnect mid-window, since
        # both accumulate a >=0 quantity every control tick) - see
        # spin_measurement_window(). True by default for fake/manual trials,
        # which have no such failure mode.
        self.last_measurement_valid = True

    def on_impulse(self, msg: WrenchStamped):
        self.latest_impulse = msg

    def on_energetics(self, msg: WrenchStamped):
        self.latest_energetics = msg

    def energetics_totals(self):
        """Returns (cumulativeEnergy_J, cumulativeCharge_mAh, msg_stamp_sec)
        from the latest micro_ros/energetics message, or (None, None, None)
        if none has arrived yet. msg_stamp_sec is the firmware's own
        header.stamp, not this node's receive time, so it matches what
        Foxglove/rosbag show when indexing by message timestamp."""
        if self.latest_energetics is None:
            return None, None, None
        stamp = self.latest_energetics.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        return (self.latest_energetics.wrench.torque.x,
                self.latest_energetics.wrench.torque.y,
                stamp_sec)

    def spin_briefly(self, duration_s):
        # Nothing spins during the input()-gated settle/tare loop in run(),
        # so self.latest_energetics (and latest_impulse) can be stale by the
        # time a trial is about to start - this flushes
        # pending callbacks so the pre-trial energetics snapshot is current,
        # not left over from the end of the previous trial.
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=max(0.0, deadline - time.monotonic()))

    def spin_measurement_window(self, duration_s):
        """Like spin_briefly(), but also watches cumulativeEnergy_J/
        cumulativeCharge_mAh for ever going backwards during the window.
        The only way that can happen is a firmware mode-switch reset or a
        reboot/reconnect mid-window - both accumulate a >=0 quantity every
        control tick (see esp32_micro_ros.ino), so they can never
        legitimately decrease. Unlike those, loadcell_data_impulse is a
        real signed integral of an oscillating force and can legitimately
        go up and down, so it's deliberately NOT checked here - this is
        energetics-only. Returns True if a violation was seen, so the
        caller can flag the whole window's numbers as untrustworthy instead
        of silently reporting a corrupted delta (this is what caught the
        negative p_bar seen in practice)."""
        EPS = 1e-6
        last_energy_J, last_charge_mAh, _ = self.energetics_totals()
        violated = False
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=max(0.0, deadline - time.monotonic()))
            energy_J, charge_mAh, _ = self.energetics_totals()
            if last_energy_J is not None and energy_J is not None and energy_J < last_energy_J - EPS:
                self.get_logger().error(
                    f'cumulativeEnergy_J went backwards mid-measurement '
                    f'({last_energy_J} -> {energy_J}) - firmware reset or reboot?')
                violated = True
            if (last_charge_mAh is not None and charge_mAh is not None
                    and charge_mAh < last_charge_mAh - EPS):
                self.get_logger().error(
                    f'cumulativeCharge_mAh went backwards mid-measurement '
                    f'({last_charge_mAh} -> {charge_mAh}) - firmware reset or reboot?')
                violated = True
            if energy_J is not None:
                last_energy_J = energy_J
            if charge_mAh is not None:
                last_charge_mAh = charge_mAh
        return violated

    def send_mode(self, mode_value: float):
        msg = Float32()
        msg.data = float(mode_value)
        self.pub_mode.publish(msg)
        self.get_logger().info(f'-> published actuator_1/mode = {mode_value}')

    def send_freq(self, freq_hz: float):
        msg = Float32()
        msg.data = float(freq_hz)
        self.pub_freq.publish(msg)
        self.get_logger().info(f'-> published actuator_1/freq = {freq_hz} Hz')

    def send_tare(self):
        msg = Bool()
        msg.data = True
        self.pub_tare.publish(msg)
        self.get_logger().info('-> published loadcell_tare_command = True')

    def start_bag_recording(self, bag_path):
        proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-a', '--exclude-regex', BAG_EXCLUDE_REGEX,
             '-o', bag_path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
        )
        # ros2 bag record needs a moment to spin up and attach to topics
        # before it's actually capturing anything - a short trial-boundary
        # recording can't afford to lose its first second the way the much
        # longer main session recording could.
        time.sleep(1.0)
        self.get_logger().info(f'Recording bag -> {bag_path}')
        return proc

    def stop_bag_recording(self, proc):
        # SIGINT (not terminate/kill) is what ros2 bag record expects for a
        # clean shutdown that properly finalizes the mcap file.
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
            self.get_logger().info('Bag recording stopped cleanly.')
        except subprocess.TimeoutExpired:
            self.get_logger().warning('Bag recorder did not exit cleanly after SIGINT - killing it.')
            proc.terminate()
            proc.wait()

    def wait_for_summary(self, freq_hz):
        # Reset every call so a fake/manual trial's row can't inherit a
        # stale measurement-window delta left over from a previous auto
        # trial - only auto_pulse_summary() ever sets these for real.
        self.last_measurement_energy_J = None
        self.last_measurement_charge_mAh = None
        self.last_measurement_t_start_sec = None
        self.last_measurement_t_end_sec = None
        self.last_measurement_valid = True

        if self.get_parameter('use_fake_summary').value:
            return self.fake_summary(freq_hz)
        if self.get_parameter('use_manual_impulse_summary').value:
            duration_s = self.get_parameter('manual_test_duration_s').value
            return self.manual_impulse_summary(duration_s)

        return self.auto_pulse_summary(freq_hz)

    def auto_pulse_summary(self, freq_hz):
        # Real automatic trial: send_freq() was already called in run(), so
        # the pulse is already running - there's no firmware trial_summary
        # to wait for (see module docstring). transient_reject_pulses run
        # first and are excluded entirely (letting the tank/motor settle
        # into steady oscillation before measuring anything); f_bar/p_bar
        # are then computed only over the following measurement_pulses, via
        # a start/end snapshot delta across just that window - NOT the
        # "whole trial total / elapsed" technique manual_impulse_summary()
        # uses, since that would let the transient contaminate the average.
        # Pulse counts, not raw seconds, so the same counts apply at every
        # frequency (period = 1/freq_hz) instead of a fixed duration
        # under/over-covering fast/slow sweeps.
        transient_pulses = self.get_parameter('transient_reject_pulses').value
        measurement_pulses = self.get_parameter('measurement_pulses').value
        period_s = 1.0 / freq_hz
        transient_s = transient_pulses * period_s
        measurement_s = measurement_pulses * period_s

        self.get_logger().info(
            f'Transient reject: {transient_s:.2f}s ({transient_pulses} pulses @ {freq_hz}Hz)...')
        # One period-length spin per pulse (rather than one big spin_briefly(
        # transient_s)) purely so each pulse can be ticked off on the
        # terminal - the total wait is identical either way.
        for pulse_num in range(1, transient_pulses + 1):
            self.get_logger().info(f'  transient pulse {pulse_num}/{transient_pulses}')
            self.spin_briefly(period_s)

        impulse_start = None if self.latest_impulse is None else self.latest_impulse.wrench.force.z
        energy_start_J, charge_start_mAh, t_start_sec = self.energetics_totals()

        self.get_logger().info(
            f'Measuring: {measurement_s:.2f}s ({measurement_pulses} pulses @ {freq_hz}Hz)...')
        # Same per-pulse split as the transient loop above, but using
        # spin_measurement_window() per chunk so the monotonicity check
        # (see its own docstring) still covers the whole measurement
        # window - self.latest_energetics keeps updating continuously in
        # the background between chunks (on_energetics doesn't care which
        # spin loop is currently polling), so splitting the wait doesn't
        # introduce any gap the check could miss a reset through.
        measurement_violated = False
        for pulse_num in range(1, measurement_pulses + 1):
            self.get_logger().info(f'  measurement pulse {pulse_num}/{measurement_pulses}')
            if self.spin_measurement_window(period_s):
                measurement_violated = True

        impulse_end = None if self.latest_impulse is None else self.latest_impulse.wrench.force.z
        self.get_logger().info(
            f'Measurement window raw impulse: impulse_start={impulse_start} impulse_end={impulse_end}')
        if impulse_start is None or impulse_end is None:
            self.get_logger().warning(
                'No loadcell_data_impulse message received - reporting 0 for f_bar')
            f_bar = 0.0
        else:
            f_bar = (impulse_end - impulse_start) / measurement_s

        energy_end_J, charge_end_mAh, t_end_sec = self.energetics_totals()
        # cumulativeEnergy_J/cumulativeCharge_mAh only ever increase in
        # firmware (both accumulate a >=0 quantity every control tick) - a
        # negative energy_end_J - energy_start_J is only possible if the
        # accumulator got reset (mode switch, or a WiFi/micro-ROS
        # reconnect reboot) between these two snapshots, not a calc bug
        # here. Logging the raw values (not just the delta) so that's
        # immediately visible instead of having to guess at it.
        self.get_logger().info(
            f'Measurement window raw energetics: '
            f'energy_start_J={energy_start_J} energy_end_J={energy_end_J} '
            f'charge_start_mAh={charge_start_mAh} charge_end_mAh={charge_end_mAh}')
        if energy_start_J is None or energy_end_J is None:
            self.get_logger().warning(
                'No micro_ros/energetics message received - reporting 0 for p_bar')
            p_bar = 0.0
        else:
            p_bar = (energy_end_J - energy_start_J) / measurement_s
            # Same measurement-window delta as p_bar, just absolute instead
            # of a rate - kept for run()'s energy_spent_J/charge_spent_mAh
            # CSV columns, see their declaration comment.
            self.last_measurement_energy_J = energy_end_J - energy_start_J
            # header.stamp of the exact messages the delta above came from -
            # for indexing a rosbag/Foxglove timeline to this window.
            self.last_measurement_t_start_sec = t_start_sec
            self.last_measurement_t_end_sec = t_end_sec
        if charge_start_mAh is not None and charge_end_mAh is not None:
            self.last_measurement_charge_mAh = charge_end_mAh - charge_start_mAh

        # A reset/reboot mid-window (see spin_measurement_window()) means
        # the whole measurement window is untrustworthy, not just whichever
        # accumulator happened to reveal it - the actuator stopped/restarted
        # mid-pulse, so f_bar is just as suspect as p_bar even though only
        # the energetics side can be checked directly (loadcell impulse is a
        # signed integral and can't use the same monotonicity check). Flag
        # with NaN (visible in the CSV/pandas) rather than a wrong-looking
        # number, and null out the energy/charge deltas too.
        if measurement_violated:
            self.get_logger().error(
                'Measurement window invalidated by a mid-window energetics '
                'reset - reporting NaN for this trial instead of a corrupted delta.')
            f_bar = math.nan
            p_bar = math.nan
            self.last_measurement_energy_J = None
            self.last_measurement_charge_mAh = None
            self.last_measurement_valid = False

        t_start_str = 'n/a' if t_start_sec is None else f'{t_start_sec:.3f}'
        t_end_str = 'n/a' if t_end_sec is None else f'{t_end_sec:.3f}'
        self.get_logger().info(
            f'Trial pulse done: f_bar={f_bar:.4f} mN, p_bar={p_bar:.4f} W '
            f'over {measurement_s:.2f}s measurement window '
            f'(energetics t_start={t_start_str}, t_end={t_end_str}, valid={not measurement_violated})')

        out = Vector3()
        out.x = f_bar
        out.y = p_bar
        out.z = 0.0
        return out

    def manual_impulse_summary(self, duration_s):
        # Real loadcell/tare/impulse, no siphon/trial_summary involved -
        # gives you a window to physically perturb the load cell by hand.
        # Reports impulse (mN*s, since the tare that just happened) divided
        # by actual elapsed wall time, i.e. an average force over the
        # window - not a real power measurement, so p_bar is just 0 here.
        print(f'\nGo - mess with the loadcell now ({int(duration_s)}s)...')
        start = time.monotonic()
        remaining = duration_s
        while remaining > 0:
            print(f'  {remaining:4.1f}s remaining...', flush=True)
            # spin_once IS the wait here, not a separate sleep() - keeps
            # on_impulse() actually receiving fresh messages throughout the
            # countdown instead of only checking once at the very end.
            rclpy.spin_once(self, timeout_sec=min(1.0, remaining))
            remaining = duration_s - (time.monotonic() - start)
        elapsed_s = time.monotonic() - start

        if self.latest_impulse is None:
            self.get_logger().warning('No loadcell_data_impulse message received - reporting 0')
            impulse_mNs = 0.0
        else:
            impulse_mNs = self.latest_impulse.wrench.force.z
        avg_force_mN = impulse_mNs / elapsed_s
        print(f'Impulse = {impulse_mNs:.4f} mN*s over {elapsed_s:.2f}s '
              f'-> avg force = {avg_force_mN:.4f} mN')
        self.last_manual_impulse_mNs = impulse_mNs
        self.last_manual_elapsed_s = elapsed_s

        out = Vector3()
        out.x = avg_force_mN
        out.y = 0.0  # no power measurement in this mode
        out.z = 0.0
        return out

    def fake_summary(self, freq_hz):
        # Arbitrary placeholder functions of frequency plus noise - not a
        # real physical model, just enough variation to sanity-check that
        # this node logs and saves a plausible-looking results table.
        out = Vector3()
        out.x = -0.5 * freq_hz + random.gauss(0, 0.02)        # fake f_bar
        out.y = 2.0 * freq_hz ** 1.5 + random.gauss(0, 0.05)  # fake p_bar
        out.z = 0.0
        self.get_logger().info(f'[FAKE] summary: f_bar={out.x:.4f}, p_bar={out.y:.4f}')
        return out

    def save_results(self, results, results_csv_path):
        # Called after every trial (not just once at the end of run()) so a
        # Ctrl+C mid-schedule - or any other crash - only loses the trial
        # in progress, not everything collected so far. Overwrites the same
        # two files each time rather than appending, so a row that changes
        # (there aren't any today, but future-proofing) or the schedule
        # itself being interrupted never leaves a stale/duplicated tail.
        df = pd.DataFrame(results)
        os.makedirs(os.path.dirname(results_csv_path), exist_ok=True)
        df.to_csv(results_csv_path, index=False)
        pickle_path = results_csv_path.replace('.csv', '.pkl')
        df.to_pickle(pickle_path)
        return df, pickle_path

    def run(self):
        frequencies = self.get_parameter('frequencies_hz').value
        replicates = self.get_parameter('replicates').value
        results_csv_path = self.get_parameter('results_csv_path').value

        resume_from = self.get_parameter('resume_from').value
        resume_index = 0
        results = []
        if resume_from:
            # Resuming writes into the ORIGINAL run's output files (not the
            # freshly-timestamped default computed above), so the trial
            # numbering/CSV picks up in place rather than forking a second
            # file - resolve results_csv_path to that original .csv path
            # regardless of whether resume_from itself points at the .csv or
            # the .pkl.
            resume_from = os.path.expanduser(resume_from)
            results_csv_path = resume_from.replace('.pkl', '.csv')
            pickle_path = results_csv_path.replace('.csv', '.pkl')
            sched_path = schedule_path_for(results_csv_path)

            if not os.path.exists(sched_path):
                raise FileNotFoundError(
                    f'Cannot resume: no schedule file at {sched_path}. '
                    f'This is written automatically alongside results_csv_path '
                    f'by every run - without it the original shuffled trial '
                    f'order cannot be reconstructed safely.')
            with open(sched_path) as f:
                schedule = [tuple(pair) for pair in json.load(f)]

            # Pickle (not CSV) so NaN/None and dtypes round-trip exactly as
            # they were written by save_results() - re-saving a CSV-parsed
            # version back out could subtly reformat already-accepted rows.
            if os.path.exists(pickle_path):
                results = pd.read_pickle(pickle_path).to_dict('records')
            elif os.path.exists(results_csv_path):
                self.get_logger().warning(
                    f'No pickle at {pickle_path} - falling back to CSV, '
                    f'NaN/None distinctions in old rows may not round-trip exactly.')
                results = pd.read_csv(results_csv_path).to_dict('records')
            else:
                raise FileNotFoundError(
                    f'Cannot resume: neither {pickle_path} nor {results_csv_path} exist.')

            resume_index = len(results)
            self.get_logger().info(
                f'Resuming from {results_csv_path}: {resume_index}/{len(schedule)} '
                f'trials already completed, continuing at trial {resume_index + 1}.')
        else:
            schedule = build_schedule(frequencies, replicates)
            self.get_logger().info(
                f'Built schedule: {len(schedule)} trials '
                f'({len(frequencies)} frequencies x {replicates} replicates)')

        # Written unconditionally (resuming or not) so this run can itself be
        # resumed later - if we just resumed, this is a same-content rewrite
        # of the schedule file we loaded above.
        sched_path = schedule_path_for(results_csv_path)
        os.makedirs(os.path.dirname(sched_path), exist_ok=True)
        with open(sched_path, 'w') as f:
            json.dump(schedule, f)

        self.send_mode(MODE_FREQ)
        record_bags = self.get_parameter('record_bags').value
        bags_dir = self.get_parameter('bags_dir').value
        if record_bags:
            os.makedirs(bags_dir, exist_ok=True)

        for i, (freq, replicate) in enumerate(schedule):
            if i < resume_index:
                continue  # already completed in the run being resumed - no re-prompt

            input(f'\n[{i + 1}/{len(schedule)}] Hit enter to start trial at {freq} Hz '
                  f'(replicate {replicate + 1}/{replicates})...')

            # Rejecting a run redoes the whole trial from the tare, not just
            # the pulse - see the "Whole trial from tare" design decision.
            # Only auto_pulse_summary() trials (not fake/manual, which have
            # no reset failure mode and no meaningful "reject" concept) go
            # through the accept/reject gate below.
            is_review_gated = (not self.get_parameter('use_fake_summary').value
                                and not self.get_parameter('use_manual_impulse_summary').value)
            attempt = 1
            skipped = False  # overwritten to True below only on an explicit skip choice
            while True:
                if attempt > 1:
                    self.get_logger().warning(
                        f'Redoing trial {i + 1}/{len(schedule)}, attempt {attempt}...')

                # Bag covers the whole trial - tare loop included, not just
                # the pulse - so a re-tare attempt or a bad tare is visible
                # in the clip too, not just the final pulsing segment.
                bag_proc = None
                bag_path = None
                if record_bags:
                    bag_path = os.path.join(
                        bags_dir,
                        f'trial_{i + 1:03d}_freq{freq}_rep{replicate + 1}_'
                        f'{datetime.now().strftime("%Y%m%d_%H%M%S")}',
                    )
                    bag_proc = self.start_bag_recording(bag_path)

                happy_with_tare = False
                while not happy_with_tare:
                    self.send_tare()
                    happy_with_tare = input('Happy with this tare? (y/n): ').strip().lower() == 'y'

                self.send_freq(freq)
                summary = self.wait_for_summary(freq)
                self.send_freq(0.0)  # explicit clean stop before the next trial's settle/tare

                # Set by auto_pulse_summary() from the same measurement-window
                # delta p_bar is computed over - see their declaration comment.
                # None for fake/manual trials, or if no micro_ros/energetics
                # message was ever received.
                energy_spent_J = self.last_measurement_energy_J
                charge_spent_mAh = self.last_measurement_charge_mAh
                measurement_t_start_sec = self.last_measurement_t_start_sec
                measurement_t_end_sec = self.last_measurement_t_end_sec
                measurement_dt_sec = (
                    None if None in (measurement_t_start_sec, measurement_t_end_sec)
                    else measurement_t_end_sec - measurement_t_start_sec)
                measurement_valid = self.last_measurement_valid

                # End-of-trial mode bounce: mode_callback() (esp32_micro_ros.ino)
                # zeroes the encoder, desiredPosition/currentPosition, and
                # cumulativeEnergy_J/cumulativeCharge_mAh on every mode command -
                # switching to POSITION and back to FREQ_SWEEP gives the next
                # trial a clean control-error/current baseline (letting the
                # position controller actually settle at zero first, not just
                # the instantaneous reset, so stiction current has time to bleed
                # off) instead of carrying over residual state from this pulse.
                self.send_mode(MODE_POSITION)
                self.spin_briefly(self.get_parameter('mode_reset_settle_s').value)
                self.send_mode(MODE_FREQ)

                if record_bags:
                    self.stop_bag_recording(bag_proc)

                energy_str = 'n/a' if energy_spent_J is None else f'{energy_spent_J:.4f}'
                charge_str = 'n/a' if charge_spent_mAh is None else f'{charge_spent_mAh:.4f}'

                if not is_review_gated:
                    break

                if not measurement_valid:
                    self.get_logger().warning(
                        'Automatic check flagged this measurement window '
                        '(energetics reset mid-window) - review before accepting.')
                print(
                    f'\nTrial {i + 1}/{len(schedule)} attempt {attempt} result: '
                    f'f_bar={summary.x:.4f} mN, p_bar={summary.y:.4f} W, '
                    f'energy_spent_J={energy_str}, charge_spent_mAh={charge_str}, '
                    f'valid={measurement_valid}')

                while True:
                    choice = input('Accept this run? [a]ccept / [r]edo / [s]kip: ').strip().lower()
                    if choice[:1] in ('a', 'r', 's'):
                        choice = choice[0]
                        break
                    print("Please enter 'a' (accept), 'r' (redo), or 's' (skip).")

                if choice == 'a':
                    skipped = False
                    break
                if choice == 's':
                    skipped = True
                    # Bag is kept (unlike redo, which discards it) - may still
                    # be useful for diagnosing why this trial got skipped, even
                    # though the numeric results aren't going in the table.
                    self.get_logger().warning(
                        f'Skipped - keeping trial {i + 1}/{len(schedule)} in the '
                        f'table with no usable data (outcome=skipped, valid=False). '
                        f'Bag file, if recorded, is kept.')
                    break

                # choice == 'r': redo
                self.get_logger().warning(
                    f'Rejected - discarding this attempt and redoing trial '
                    f'{i + 1}/{len(schedule)} from the tare.')
                if record_bags and bag_path and os.path.isdir(bag_path):
                    shutil.rmtree(bag_path, ignore_errors=True)
                attempt += 1

            # Skipping discards this attempt's numbers entirely - NaN/None
            # rather than whatever the last (rejected) attempt happened to
            # measure, so a skipped row can never silently leak into a mean/
            # plot. 'valid' stays False (reuses plot_test_results.py's
            # existing valid-row filter with no changes needed there);
            # 'outcome' separately records that this was an operator choice,
            # not (just) the automatic energetics-reset check.
            if skipped:
                f_bar_out = math.nan
                p_bar_out = math.nan
                energy_spent_J = None
                charge_spent_mAh = None
                measurement_t_start_sec = None
                measurement_t_end_sec = None
                measurement_dt_sec = None
                row_valid = False
            else:
                f_bar_out = summary.x
                p_bar_out = summary.y
                row_valid = measurement_valid

            result_row = {
                'frequency_hz': freq,
                'replicate': replicate,
                'f_bar': f_bar_out,
                'p_bar': p_bar_out,
                'energy_spent_J': energy_spent_J,
                'charge_spent_mAh': charge_spent_mAh,
                'measurement_t_start_sec': measurement_t_start_sec,
                'measurement_t_end_sec': measurement_t_end_sec,
                'measurement_dt_sec': measurement_dt_sec,
                'valid': row_valid,
                'outcome': 'skipped' if skipped else 'accepted',
                'bag_path': bag_path,
            }
            if self.get_parameter('use_manual_impulse_summary').value:
                result_row['impulse_mNs'] = self.last_manual_impulse_mNs
                result_row['elapsed_s'] = self.last_manual_elapsed_s
            results.append(result_row)
            self.get_logger().info(
                f'Trial {i + 1}/{len(schedule)} done ({result_row["outcome"]}): '
                f'f_bar={f_bar_out:.4f}, p_bar={p_bar_out:.4f}, '
                f'energy_spent_J={energy_str}, charge_spent_mAh={charge_str}')
            # Save after every trial, not just at the end - see
            # save_results()'s declaration comment for why.
            self.save_results(results, results_csv_path)

        # End-of-schedule cleanup, same reasoning as the per-trial mode
        # bounce above - explicit zero-frequency stop plus a POSITION/
        # FREQ_SWEEP bounce to reset cumulativeEnergy_J/cumulativeCharge_mAh
        # one last time, so the hardware isn't left mid-cumulative after the
        # final trial's own reset already happened minutes ago.
        self.send_freq(0.0)
        self.send_mode(MODE_POSITION)
        self.spin_briefly(self.get_parameter('mode_reset_settle_s').value)
        self.send_mode(MODE_FREQ)

        df, pickle_path = self.save_results(results, results_csv_path)
        self.get_logger().info(
            f'Saved {len(df)} trial results -> {results_csv_path} and {pickle_path}')


def main():
    rclpy.init()
    node = TestRunnerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
