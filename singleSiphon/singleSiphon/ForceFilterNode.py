import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped

best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)


class ForceFilterNode(Node):
    """Kalman-filters loadcell_data (wrench.force.z) and republishes it.

    State is [force, d(force)/dt] (constant-velocity model) so the filter
    can track a moving force reading, not just denoise a static one. Process
    noise is built from a single "how fast can force plausibly accelerate"
    parameter (continuous white-noise-acceleration model); measurement noise
    is the loadcell's own reading variance. Both are just starting guesses -
    see the covariance-estimation script for how to derive real values from
    logged data instead of guessing.
    """

    def __init__(self):
        super().__init__('force_filter')

        # Rescaled from the original raw-count-scale tuning (3e6 / 1000.0)
        # now that esp32LoadCell_mROS.ino publishes /loadcell_data already
        # calibrated to millinewtons (slope=-0.08355) instead of raw counts.
        # measurement_noise_variance is a VARIANCE -> scales by slope^2;
        # process_noise_accel_stddev is a STDDEV -> scales by |slope|. This
        # is the linear-transform math, not a fresh derivation from real
        # mN-scale data - re-run estimate_force_noise.py against a real
        # static segment once one exists at the new scale, same as the
        # original values were derived.
        self.declare_parameter('measurement_noise_variance', 20941.8)
        self.declare_parameter('process_noise_accel_stddev', 83.55)

        self.sub = self.create_subscription(
            WrenchStamped, 'loadcell_data', self.on_measurement, best_effort_qos)
        self.pub = self.create_publisher(
            WrenchStamped, 'loadcell_data_filtered', best_effort_qos)

        self.x = None  # [force, force_rate]
        self.P = None  # state covariance
        self.last_stamp_ns = None

        # Received-count logging: BEST_EFFORT QoS at ~2kHz over live DDS can
        # silently drop messages under load (rosbag2_player + foxglove_bridge
        # + this node all competing), which an offline read straight from
        # the bag file never experiences - that's a real, easy way for this
        # node's output to diverge from tune_filter_offline.py's even with
        # identical R/Q. Compare this count against `ros2 bag info`'s
        # message count for /loadcell_data on the same bag.
        self.received_count = 0
        self.create_timer(5.0, self.log_received_count)

    def log_received_count(self):
        self.get_logger().info(f'Received {self.received_count} /loadcell_data messages so far')

    def on_measurement(self, msg: WrenchStamped):
        self.received_count += 1
        z = msg.wrench.force.z
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if self.x is None:
            self.x = np.array([z, 0.0])
            self.P = np.eye(2)
            self.last_stamp_ns = stamp_ns
            self.publish(msg, self.x[0])
            return

        dt = (stamp_ns - self.last_stamp_ns) / 1e9
        self.last_stamp_ns = stamp_ns
        if dt <= 0:
            # Out-of-order or duplicate timestamp - skip the predict step
            # rather than divide/propagate with a bogus or negative dt.
            dt = 1e-6

        q_accel = self.get_parameter('process_noise_accel_stddev').value ** 2
        r_meas = self.get_parameter('measurement_noise_variance').value

        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        # Discretized continuous white-noise-acceleration process noise.
        Q = q_accel * np.array([[dt**3 / 3, dt**2 / 2],
                                 [dt**2 / 2, dt]])
        H = np.array([[1.0, 0.0]])
        R = np.array([[r_meas]])

        # MATLAB equivalent of the predict/update below, given x (2x1),
        # P (2x2), F, Q, H (1x2), R (scalar), z (scalar measurement):
        #
        #   % Predict
        #   x_pred = F * x;
        #   P_pred = F * P * F' + Q;
        #
        #   % Update
        #   y = z - H * x_pred;
        #   S = H * P_pred * H' + R;
        #   K = P_pred * H' / S;
        #   x = x_pred + K * y;
        #   P = P_pred - K * H * P_pred;

        # Predict
        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q

        # Update
        y = z - (H @ x_pred)[0]
        S = (H @ P_pred @ H.T)[0, 0] + R[0, 0]
        K = (P_pred @ H.T) / S
        self.x = x_pred + (K.flatten() * y)
        self.P = P_pred - K @ H @ P_pred

        self.publish(msg, self.x[0])

    def publish(self, source_msg: WrenchStamped, filtered_force: float):
        out = WrenchStamped()
        out.header = source_msg.header
        out.wrench.force.z = filtered_force
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ForceFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
