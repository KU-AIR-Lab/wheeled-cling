#!/usr/bin/env python3
"""Convert geometry_msgs/Twist to geometry_msgs/TwistStamped for swerve_controller."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.declare_parameter('frame_id', 'base_link')
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_in', self._cmd_vel_cb, 10)
        self.get_logger().info('cmd_vel_stamper ready: Twist -> TwistStamped')

    def _cmd_vel_cb(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
