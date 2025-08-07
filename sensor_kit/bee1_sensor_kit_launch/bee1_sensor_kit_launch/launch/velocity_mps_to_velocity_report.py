#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from autoware_auto_vehicle_msgs.msg import VelocityReport

class VelocityMpsToVelocityReport(Node):
    def __init__(self):
        super().__init__('velocity_mps_to_velocity_report')
        self.sub = self.create_subscription(
            Float32,
            '/beemobs/FB_VehicleSpeed',
            self.speed_callback,
            10)
        self.pub = self.create_publisher(
            VelocityReport,
            '/vehicle/status/velocity_status',
            10)

    def speed_callback(self, msg):
        report = VelocityReport()
        report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = 'base_link'
        report.longitudinal_velocity = msg.data
        report.lateral_velocity = 0.0
        report.heading_rate = 0.0
        self.pub.publish(report)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityMpsToVelocityReport()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
