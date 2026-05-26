#!/usr/bin/env python3
"""
odom_node.py  —  Odometry publisher for the 3D3S robot.

Subscribes to /joint_states, runs forward kinematics (dead-reckoning),
and publishes:
  - nav_msgs/Odometry  on  /odom
  - TF:  odom → base_footprint

This is the bridge between joint-level sensing and the nav2 navigation stack.
Works in both RViz mode (joint_states from cmd_vel_to_wheels) and Gazebo mode
(joint_states from joint_state_broadcaster).

Usage
-----
  ros2 run robot_3d3s odom_node.py
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ── Robot geometry (must match motion_demo.py / teleop_keyboard.py) ───────────

R_WHEEL   = 0.050
R_LEG     = 0.38905
R_CONTACT = R_LEG + 0.025
LEG_ANGLES = [0.0, 2 * math.pi / 3, 4 * math.pi / 3]
P_CONTACT  = [
    (R_CONTACT * math.cos(a), R_CONTACT * math.sin(a))
    for a in LEG_ANGLES
]

STEER_JOINTS = ['steering_1_joint', 'steering_2_joint', 'steering_3_joint']
WHEEL_JOINTS  = ['wheel_1_joint',   'wheel_2_joint',    'wheel_3_joint'   ]

ODOM_FRAME  = 'odom'
ROBOT_FRAME = 'base_footprint'


# ── Kinematics ────────────────────────────────────────────────────────────────

def _body_velocity(steer, wheel_vel):
    """Forward kinematics: return [vx, vy, ωz] in the body frame."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for i, (a, s, wv, (px, py)) in enumerate(
            zip(LEG_ANGLES, steer, wheel_vel, P_CONTACT)):
        phi = a - s
        sp, cp = math.sin(phi), math.cos(phi)
        A[i] = [-sp, cp, sp * py + cp * px]
        b[i] = -wv * R_WHEEL
    v, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return v


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


# ── Node ─────────────────────────────────────────────────────────────────────

class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_br    = TransformBroadcaster(self)

        self._x = self._y = self._theta = 0.0
        self._last_t = None

        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.get_logger().info(
            'Odom node started — publishing /odom and TF odom→base_footprint')

    def _js_cb(self, msg: JointState):
        name_map = {n: i for i, n in enumerate(msg.name)}

        try:
            steer = [msg.position[name_map[n]] for n in STEER_JOINTS]
            wvel  = [msg.velocity[name_map[n]] for n in WHEEL_JOINTS]
        except (KeyError, IndexError):
            return  # joint_states not fully populated yet

        now = msg.header.stamp

        if self._last_t is None:
            self._last_t = now
            return

        dt = (now.sec - self._last_t.sec) + \
             (now.nanosec - self._last_t.nanosec) * 1e-9
        self._last_t = now

        if dt <= 0.0 or dt > 0.5:
            return

        # Forward kinematics → body velocity
        v  = _body_velocity(steer, wvel)
        vx, vy, wz = float(v[0]), float(v[1]), float(v[2])

        # Integrate world pose (dead-reckoning)
        ct = math.cos(self._theta)
        st = math.sin(self._theta)
        self._x     += (ct * vx - st * vy) * dt
        self._y     += (st * vx + ct * vy) * dt
        self._theta += wz * dt

        qx, qy, qz, qw = _yaw_to_quat(self._theta)

        # ── Publish nav_msgs/Odometry ─────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id  = ROBOT_FRAME

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz

        # Diagonal covariance — modest uncertainty for dead-reckoning
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.02   # yaw
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.01
        odom.twist.covariance[35] = 0.02

        self._odom_pub.publish(odom)

        # ── Broadcast TF: odom → base_footprint ──────────────────────────────
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id  = ROBOT_FRAME
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_br.sendTransform(t)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
