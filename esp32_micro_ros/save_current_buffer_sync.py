#!/usr/bin/env python3
"""
Waits for the ESP32 telemetry publisher, then sits idle until a manual trigger
(publish std_msgs/Bool {data: true} to actuator_1/save_buffer_trigger, e.g. from a
Foxglove Publish panel) requests a dump. Each dump saves all three channels (current
ADC, actual shaft position, desired shaft position) to a timestamped .npz file.

Trigger protocol: publishes Float32(data=2.0) to actuator_1/dump_current_buffer.
  1.0 → current ADC buffer only (old single-channel dump, not handled here)
  2.0 → telemetry: interleaved [current, actual_pos, desired_pos] triplets

Missing chunks (best-effort UDP, no delivery guarantee): once the dump goes idle,
any missing seq numbers are re-requested by publishing (seq + RETRANSMIT_TELEM_OFFSET)
to actuator_1/retransmit_chunk, up to MAX_RETRANSMIT_ROUNDS times. That's the same
topic/subscriber the old current-buffer-only dump uses for retransmits — the ESP32's
micro-ROS build caps out at 5 subscriptions (baked into its prebuilt libmicroros.a),
so telemetry retransmits are dispatched through it via an offset rather than a
subscriber of their own (mirrors dump_trigger_callback's own 1.0-vs-2.0 dispatch).
Whatever's still missing after all rounds is saved as a gap (valid=False) rather
than held forever.

Chunk format:
  Chunk 0:    [0=seq, 1-4=anchor_ns (4×u16, MSW first), 5=rate_hz, 6,7,8=triplet0, ...]
  Non-zero:   [0=seq, 1,2,3=triplet0, 4,5,6=triplet1, ...]
  Triplet:    (current u16 ADC, actual_pos i16 encoder ticks, desired_pos i16 encoder ticks)
  Scale:      ticks × TICKS_TO_MDEG (= 360000 / CPR / SR) → millidegrees

Saved .npz keys:
  current      uint16  raw ADC counts (0-4095)
  actual_pos   float32 shaft position in millidegrees
  desired_pos  float32 desired shaft position in millidegrees
  valid        bool    False where a chunk went missing and the sample was gap-filled
                        with 0 rather than real data — check this before trusting a sample
  timestamps   float64 ROS epoch seconds, one per sample
  sample_rate_hz, ros_anchor_ns, ticks_to_mdeg

Usage:
    python3 save_current_buffer_sync.py              # saves to telemetry_buffer_<timestamp>.npz
    python3 save_current_buffer_sync.py my_run        # saves to my_run_<timestamp>.npz
    # Re-trigger without restarting: publish std_msgs/Bool {data: true}
    # to actuator_1/save_buffer_trigger (e.g. from a Foxglove Publish panel).
    # Every save — the initial one and each re-trigger — uses the same prefix,
    # so all of a session's dumps are obviously one family of files.
"""

import os
import sys
import time
from datetime import datetime, timezone
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt16MultiArray, Float32, Bool

IDLE_TIMEOUT_S        = 3.0
NO_RESPONSE_TIMEOUT_S = 5.0
MAX_RETRANSMIT_ROUNDS = 5  # give up and save with gaps after this many rounds
BEST_EFFORT_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# Must match ESP32 firmware's RETRANSMIT_TELEM_OFFSET — the shared retransmit
# subscriber dispatches on this: values below it are current-buffer chunk seqs,
# values at/above it are telemetry chunk seqs (value - offset).
RETRANSMIT_TELEM_OFFSET = 1_000_000

# Must match ESP32 firmware: 360000.0 / CPR / SR  (CPR=12, SR=100.37)
TICKS_TO_MDEG = 360000.0 / 12 / 100.37

# Must match ESP32 firmware: DUMP_CHUNK_SIZE=240, CHUNK_DATA_SAMPLES=235, /3 triplets/chunk.
# Every full chunk (including chunk 0, despite its extra header words) carries exactly
# this many triplets — the header occupies separate array slots, it doesn't steal from
# the payload. Used to place each chunk at its true buffer offset (seq * this) so a
# missing chunk leaves a real gap instead of shifting every later sample's timestamp.
TELEM_SAMPLES_PER_CHUNK = (240 - 5) // 3


class TelemetrySaver(Node):
    def __init__(self, prefix: str):
        super().__init__('telemetry_buffer_saver')
        self.prefix           = prefix
        self.output_path      = f'{prefix}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.npz'
        self.received: dict[int, tuple] = {}  # seq -> (current, actual_pos, desired_pos) ndarrays
        self.max_seq          = -1
        self.last_msg_time    = time.monotonic()
        self.done             = False
        self.busy             = False
        self._publisher_seen  = False
        self.retransmit_round = 0

        self.ros_anchor_ns: int | None = None
        self.sample_rate_hz: int = 1000

        self.sub = self.create_subscription(
            UInt16MultiArray, 'actuator_1/telemetry_buffer', self._cb,
            qos_profile=BEST_EFFORT_QOS
        )
        self.dump_trigger_pub = self.create_publisher(
            Float32, 'actuator_1/dump_current_buffer', qos_profile=BEST_EFFORT_QOS
        )
        self.retransmit_pub = self.create_publisher(
            Float32, 'actuator_1/retransmit_chunk', qos_profile=BEST_EFFORT_QOS
        )
        self.trigger_sub = self.create_subscription(
            Bool, 'actuator_1/save_buffer_trigger', self._on_trigger,
            qos_profile=BEST_EFFORT_QOS
        )

        self.create_timer(0.5, self._check_idle)
        self._ready_timer = self.create_timer(0.5, self._wait_for_publisher)
        self.get_logger().info('Waiting for ESP32 telemetry publisher...')

    def _wait_for_publisher(self):
        if self.count_publishers('actuator_1/telemetry_buffer') > 0:
            self._ready_timer.cancel()
            self._publisher_seen = True
            self.get_logger().info('Publisher found — ready for manual trigger.')

    def _on_trigger(self, msg: Bool):
        if not msg.data:
            return
        if not self._publisher_seen:
            self.get_logger().warn('Trigger ignored — ESP32 telemetry publisher not seen yet.')
            return
        if self.busy:
            self.get_logger().warn('Trigger ignored — a save is already in progress.')
            return
        self.get_logger().info('Trigger received — starting new save.')
        self._reset(f'{self.prefix}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.npz')
        self._trigger_dump()

    def _reset(self, path: str):
        self.output_path     = path
        self.received        = {}
        self.max_seq         = -1
        self.done            = False
        self.busy            = True
        self.retransmit_round = 0
        self.ros_anchor_ns   = None
        self.last_msg_time   = time.monotonic()

    def _trigger_dump(self):
        self.busy = True
        self.last_msg_time = time.monotonic()
        msg = Float32()
        msg.data = 2.0  # telemetry dump
        self.dump_trigger_pub.publish(msg)
        self.get_logger().info('Telemetry dump triggered.')

    def _cb(self, msg: UInt16MultiArray):
        data = msg.data
        if len(data) < 4:
            return

        seq = int(data[0])

        if seq == 0:
            # [0=seq, 1-4=anchor_ns, 5=rate, 6,7,8=triplet0, ...]
            if len(data) < 9:
                self.get_logger().warn('Chunk 0 too short — dropping')
                return
            self.ros_anchor_ns = (
                (int(data[1]) << 48) | (int(data[2]) << 32) |
                (int(data[3]) << 16) |  int(data[4])
            )
            self.sample_rate_hz = int(data[5])
            payload = np.array(data[6:], dtype=np.uint16)
        else:
            # [0=seq, 1,2,3=triplet0, ...]
            payload = np.array(data[1:], dtype=np.uint16)

        # Split interleaved triplets; trim any trailing partial triplet
        n = (len(payload) // 3) * 3
        payload = payload[:n]
        cur  = payload[0::3]
        apos = (payload[1::3].view(np.int16).astype(np.float32)) * TICKS_TO_MDEG
        dpos = (payload[2::3].view(np.int16).astype(np.float32)) * TICKS_TO_MDEG

        if seq in self.received:
            return
        self.received[seq] = (cur, apos, dpos)
        self.max_seq = max(self.max_seq, seq)
        self.last_msg_time = time.monotonic()
        total = sum(len(v[0]) for v in self.received.values())
        if seq == 0:
            self.get_logger().info(
                f'Chunk 0: anchor={self.ros_anchor_ns} ns  rate={self.sample_rate_hz} Hz  '
                f'{len(cur)} triplets'
            )
        else:
            self.get_logger().info(f'  chunk {seq}: {len(cur)} triplets  (total: {total})')

    def _check_idle(self):
        if self.done or not self.busy:
            return
        if self.max_seq < 0:
            if time.monotonic() - self.last_msg_time >= NO_RESPONSE_TIMEOUT_S:
                self.get_logger().warn(
                    f'No chunks received within {NO_RESPONSE_TIMEOUT_S}s — '
                    'ESP32 likely never got the dump command.'
                )
                self.done = True
                self.busy = False
                self.get_logger().info('Ready for next trigger.')
            return
        if time.monotonic() - self.last_msg_time >= IDLE_TIMEOUT_S:
            missing = [i for i in range(self.max_seq + 1) if i not in self.received]
            if missing and self.retransmit_round < MAX_RETRANSMIT_ROUNDS:
                self.retransmit_round += 1
                self.get_logger().warn(
                    f'{len(missing)} missing chunk(s) — requesting retransmit '
                    f'(round {self.retransmit_round}/{MAX_RETRANSMIT_ROUNDS}): '
                    f'{missing[:10]}{"..." if len(missing) > 10 else ""}'
                )
                for seq in missing:
                    msg = Float32()
                    msg.data = float(seq + RETRANSMIT_TELEM_OFFSET)
                    self.retransmit_pub.publish(msg)
                self.last_msg_time = time.monotonic()  # give this round its own idle window
                return
            if missing:
                self.get_logger().warn(
                    f'Saving with {len(missing)} still-missing chunks after '
                    f'{MAX_RETRANSMIT_ROUNDS} retransmit rounds: '
                    f'{missing[:10]}{"..." if len(missing) > 10 else ""}'
                )
            self._save()

    def _save(self):
        if self.done:
            return
        self.done = True

        # Place each chunk at its true buffer offset rather than concatenating in
        # receipt order — a missing chunk then leaves a real gap (valid=False)
        # instead of shifting every later sample's timestamp by one chunk's worth.
        n_total = (self.max_seq + 1) * TELEM_SAMPLES_PER_CHUNK
        current     = np.zeros(n_total, dtype=np.uint16)
        actual_pos  = np.zeros(n_total, dtype=np.float32)
        desired_pos = np.zeros(n_total, dtype=np.float32)
        valid       = np.zeros(n_total, dtype=bool)
        for seq, (cur, apos, dpos) in self.received.items():
            start = seq * TELEM_SAMPLES_PER_CHUNK
            n = len(cur)
            current[start:start + n]     = cur
            actual_pos[start:start + n]  = apos
            desired_pos[start:start + n] = dpos
            valid[start:start + n]       = True

        if self.ros_anchor_ns is not None:
            dt = 1.0 / self.sample_rate_hz
            timestamps = self.ros_anchor_ns * 1e-9 + np.arange(n_total) * dt
            self.get_logger().info(
                f'Time anchor: {self.ros_anchor_ns} ns  '
                f'({datetime.fromtimestamp(self.ros_anchor_ns * 1e-9, tz=timezone.utc).isoformat()})'
            )
        else:
            self.get_logger().warn('No time anchor — timestamps will be sample-index only')
            timestamps = np.arange(n_total, dtype=np.float64)

        out_path = self.output_path
        if not out_path.endswith('.npz'):
            out_path += '.npz'

        np.savez(
            out_path,
            current=current,
            actual_pos=actual_pos,
            desired_pos=desired_pos,
            valid=valid,
            timestamps=timestamps,
            sample_rate_hz=np.array(self.sample_rate_hz),
            ros_anchor_ns=np.array(self.ros_anchor_ns or 0, dtype=np.int64),
            ticks_to_mdeg=np.array(TICKS_TO_MDEG),
        )
        n_missing = n_total - int(valid.sum())
        duration = n_total / self.sample_rate_hz
        self.get_logger().info(
            f'Saved {n_total} samples ({duration:.1f}s, {n_missing} gap-filled) -> {out_path}'
        )

        try:
            from npz_to_bag import npz_to_bag
            bag_path = npz_to_bag(out_path)

            # rosbag2 writes <bag_path>/<basename(bag_path)>_0.mcap. Hard-link that
            # up as a flat sibling next to the npz so it can be multi-selected
            # alongside the main recording's .mcap in Foxglove's file-open dialog.
            # (A symlink looks right to `ls` but Foxglove's file picker doesn't
            # follow it — a hard link is indistinguishable from a real file.)
            mcap_name = f'{os.path.basename(bag_path)}_0.mcap'
            link_path = os.path.splitext(out_path)[0] + '.mcap'
            if os.path.lexists(link_path):
                os.remove(link_path)
            os.link(os.path.join(bag_path, mcap_name), link_path)

            self.get_logger().info(f'Bag written -> {bag_path}  (hard link: {link_path})')
        except Exception as e:
            self.get_logger().warn(f'npz_to_bag failed: {e}')

        self.busy = False
        self.get_logger().info('Ready for next trigger.')


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else 'telemetry_buffer'
    rclpy.init()
    node = TelemetrySaver(prefix)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.received and not node.done:
            node._save()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
