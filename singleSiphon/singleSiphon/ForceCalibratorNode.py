import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped

best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)


class ForceCalibratorNode(Node):
    """
    This subscribes to /loadcell_data_filtered, applies a linear calibration (slope only), and republishes the calibrated force on /loadcell_data_calibrated.
    The calibration parameter can be set via the ROS2 parameter 'calibration_slope'.

    slope	     -0.08355

    No intercept: dropped on the assumption that it reflected the original
    calibration run's taring procedure rather than something intrinsic to
    the sensor - see esp32LoadCell_mROS.ino's calibration comment for the
    full reasoning and the caveat about redoing this as a proper
    through-origin regression if that assumption turns out wrong.
    """

    def __init__(self):
        super().__init__('force_calibrator')

        self.declare_parameter('calibration_slope', -0.08355)

        self.sub = self.create_subscription(
            WrenchStamped, 'loadcell_data_filtered', self.on_measurement, best_effort_qos)
        self.pub = self.create_publisher(
            WrenchStamped, 'loadcell_data_calibrated', best_effort_qos)

        self.received_count = 0
        self.create_timer(5.0, self.log_received_count)

    def log_received_count(self):
        self.get_logger().info(f'Calibrated {self.received_count} /loadcell messages so far')

    def on_measurement(self, msg: WrenchStamped):
        self.received_count += 1
        z = msg.wrench.force.z
        
        slope = self.get_parameter('calibration_slope').get_parameter_value().double_value
        calibrated_force = slope * z


        self.publish(msg,calibrated_force)

    def publish(self, source_msg: WrenchStamped, filtered_force: float):
        out = WrenchStamped()
        out.header = source_msg.header
        out.wrench.force.z = filtered_force
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ForceCalibratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
