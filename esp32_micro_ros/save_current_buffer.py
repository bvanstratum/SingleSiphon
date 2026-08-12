#!/usr/bin/env python3
"""
Waits for ESP32 chunk publisher, auto-triggers the dump, collects chunks, saves to .npy.
remind me to rewrite this for receiving the timine headers

Usage:
    python3 save_current_buffer.py              # saves to current_buffer_<timestamp>.npy
    python3 save_current_buffer.py my_run.npy
"""

import sys
import time
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt16MultiArray, Float32

IDLE_TIMEOUT_S   = 3.0
RETRANSMIT_WAIT  = 1.0   # seconds to wait after blasting requests before checking again
MAX_ROUNDS       = 5     # give up after this many blast rounds
SAMPLE_RATE_HZ   = 1000
BEST_EFFORT_QOS  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class BufferSaver(Node):
    def __init__(self, output_path: str):
        super().__init__('current_buffer_saver')
        self.output_path    = output_path
        self.received: dict[int, np.ndarray] = {}  # seq -> samples
        self.max_seq        = -1
        self.last_msg_time  = time.monotonic()
        self.done           = False
        self._retransmit_round = 0

        self.sub = self.create_subscription(
            UInt16MultiArray, 'actuator_1/current_buffer', self._cb, qos_profile=BEST_EFFORT_QOS
        )
        self.retransmit_pub = self.create_publisher(
            Float32, 'actuator_1/retransmit_chunk', qos_profile=BEST_EFFORT_QOS
        )
        self.dump_trigger_pub = self.create_publisher(
            Float32, 'actuator_1/dump_current_buffer', qos_profile=BEST_EFFORT_QOS
        )
        self.create_timer(0.5, self._check_idle)
        self._ready_timer = self.create_timer(0.5, self._wait_for_publisher)
        self.get_logger().info('Waiting for ESP32 chunk publisher...')

    def _wait_for_publisher(self):
        if self.count_publishers('actuator_1/current_buffer') > 0:
            self._ready_timer.cancel()
            self.get_logger().info('Publisher found — waiting 1.5s for DDS to negotiate...')
            self._trigger_timer = self.create_timer(1.5, self._trigger_dump)

    def _trigger_dump(self):
        self._trigger_timer.cancel()
        msg = Float32()
        msg.data = 1.0
        self.dump_trigger_pub.publish(msg)
        self.get_logger().info('Dump triggered.')

    def _cb(self, msg: UInt16MultiArray):
        if len(msg.data) < 2:
            return
        seq   = int(msg.data[0])
        chunk = np.array(msg.data[1:], dtype=np.uint16)
        if seq in self.received:
            return
        self.received[seq] = chunk
        self.max_seq = max(self.max_seq, seq)
        self.last_msg_time = time.monotonic()
        total = sum(len(v) for v in self.received.values())
        self.get_logger().info(f'  chunk {seq}: {len(chunk)} samples  (total: {total})')

    def _check_idle(self):
        if self.done or self.max_seq < 0:
            return
        if time.monotonic() - self.last_msg_time >= IDLE_TIMEOUT_S:
            missing = [i for i in range(self.max_seq + 1) if i not in self.received]
            if not missing:
                self._save()
                return
            if self._retransmit_round >= MAX_ROUNDS:
                self.get_logger().warn(f'Giving up — {len(missing)} chunks still missing after {MAX_ROUNDS} rounds')
                self._save()
                return
            self._retransmit_round += 1
            self.get_logger().warn(f'Round {self._retransmit_round}/{MAX_ROUNDS}: blasting {len(missing)} retransmit requests...')
            for seq in missing:
                req = Float32()
                req.data = float(seq)
                self.retransmit_pub.publish(req)
            # Reset idle clock so we wait another window for responses
            self.last_msg_time = time.monotonic()

    def _retransmit_next(self):
        pass  # unused — kept for compatibility

    def _save(self):
        if self.done:
            return
        self.done = True
        missing = [i for i in range(self.max_seq + 1) if i not in self.received]
        if missing:
            self.get_logger().warn(f'Saving with {len(missing)} missing chunks: {missing[:10]}{"..." if len(missing) > 10 else ""}')
        ordered = [self.received[i] for i in sorted(self.received.keys())]
        data = np.concatenate(ordered)
        np.save(self.output_path, data)
        self.get_logger().info(
            f'Saved {len(data)} samples ({len(data)/SAMPLE_RATE_HZ:.1f}s) -> {self.output_path}'
        )
        raise SystemExit(0)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else f'current_buffer_{datetime.now().strftime("%Y%m%d_%H%M%S")}.npy'
    rclpy.init()
    node = BufferSaver(path)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        if node.received and not node.done:
            node._save()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

