#!/usr/bin/env python3
"""
cmd_vel_to_wheels.py  —  Velocity bridge for the 3D3S robot.

Converts geometry_msgs/Twist on /cmd_vel into steering position commands and
wheel velocity commands using the robot's inverse kinematics.

Designed to work with:
  - nav2  (autonomous navigation — nav2 publishes /cmd_vel)
  - teleop_twist_keyboard  (manual keyboard control)
  - Any ROS2 node that publishes geometry_msgs/Twist to /cmd_vel

Features
--------
  Smart flip:  avoids unnecessary re-steering by flipping wheel direction.
  Steering-first gate:  wheels held at zero while steering transitions.

Modes
-----
RViz mode (default):   publishes /joint_states
  ros2 run robot_3d3s cmd_vel_to_wheels.py

Gazebo mode:           publishes to ros2_control controllers
  ros2 run robot_3d3s cmd_vel_to_wheels.py --ros-args -p gazebo:=true
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ── Robot geometry ────────────────────────────────────────────────────────────

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

STEER_RATE      = 1.2              # rad/s — steering ramp speed (RViz mode)
STEER_THRESHOLD = math.radians(5)  # wheels wait until steering error < 5°
PUBLISH_HZ      = 50


# ── Kinematics ────────────────────────────────────────────────────────────────

def _ang_dist(a, b):
    d = abs(a - b) % (2 * math.pi)
    return min(d, 2 * math.pi - d)


def _inverse_kinematics(vx, vy, omega):
    """Raw IK: steering angles in [-π/2, π/2], negative speed = forward."""
    steers, speeds = [], []
    for a_i in LEG_ANGLES:
        vwx = vx - math.sin(a_i) * omega * R_CONTACT
        vwy = vy + math.cos(a_i) * omega * R_CONTACT
        mag = math.hypot(vwx, vwy)
        if mag < 1e-9:
            steers.append(0.0)
            speeds.append(0.0)
            continue
        rolling_dir = math.atan2(vwy, vwx)
        phi = rolling_dir - math.pi / 2
        s   = a_i - phi
        s   = (s + math.pi) % (2 * math.pi) - math.pi
        sign = 1.0
        if s > math.pi / 2:
            s -= math.pi; sign = -1.0
        elif s < -math.pi / 2:
            s += math.pi; sign = -1.0
        steers.append(s)
        speeds.append(-sign * mag / R_WHEEL)
    return steers, speeds


def _smart_commands(vx, vy, omega, current_steer):
    """IK with minimal-steering optimisation (smart flip)."""
    if abs(vx) + abs(vy) + abs(omega) < 1e-9:
        return list(current_steer), [0.0, 0.0, 0.0]

    raw_steers, raw_speeds = _inverse_kinematics(vx, vy, omega)
    steers, speeds = [], []
    for s_new, w_new, s_cur in zip(raw_steers, raw_speeds, current_steer):
        if abs(w_new) < 1e-9:
            steers.append(s_cur); speeds.append(0.0); continue
        s_flip = s_new - math.pi if s_new > 0 else s_new + math.pi
        if _ang_dist(s_flip, s_cur) < _ang_dist(s_new, s_cur):
            steers.append(s_flip); speeds.append(-w_new)
        else:
            steers.append(s_new); speeds.append(w_new)
    return steers, speeds


# ── Node ─────────────────────────────────────────────────────────────────────

class CmdVelToWheels(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_wheels')

        self.declare_parameter('gazebo', False)
        self.gazebo_mode = self.get_parameter('gazebo').value

        # Publishers
        if self.gazebo_mode:
            self.steer_pub = self.create_publisher(
                Float64MultiArray, '/steering_controller/commands', 10)
            self.wheel_pub = self.create_publisher(
                Float64MultiArray, '/wheel_controller/commands', 10)
            self.get_logger().info('Gazebo mode')
        else:
            self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.get_logger().info('RViz mode')

        # Desired velocity from /cmd_vel
        self._vx = self._vy = self._omega = 0.0
        self._vel_lock = threading.Lock()

        # Tracked steering state (actual in Gazebo, ramped in RViz)
        self._steer     = [0.0, 0.0, 0.0]
        self._wheel_pos = [0.0, 0.0, 0.0]   # RViz animation only

        # In Gazebo mode: read actual steering from joint_states
        if self.gazebo_mode:
            self.create_subscription(
                JointState, '/joint_states', self._js_cb, 10)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)

        self.get_logger().info(
            'cmd_vel_to_wheels ready — listening on /cmd_vel')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        with self._vel_lock:
            self._vx    = msg.linear.x
            self._vy    = msg.linear.y
            self._omega = msg.angular.z

    def _js_cb(self, msg: JointState):
        """Gazebo only: track actual steering positions."""
        name_map = {n: i for i, n in enumerate(msg.name)}
        try:
            self._steer = [msg.position[name_map[n]] for n in STEER_JOINTS]
        except (KeyError, IndexError):
            pass

    # ── 50 Hz publish ─────────────────────────────────────────────────────────

    def _publish_cb(self):
        dt = 1.0 / PUBLISH_HZ
        with self._vel_lock:
            vx, vy, omega = self._vx, self._vy, self._omega

        target_steer, wheel_speeds = _smart_commands(vx, vy, omega, self._steer)

        # Steering-first gate
        max_err = max(_ang_dist(target_steer[i], self._steer[i]) for i in range(3))
        steering_busy = max_err > STEER_THRESHOLD

        if self.gazebo_mode:
            # In Gazebo the position controller handles the ramp internally.
            # We just publish the target and withhold wheel commands if needed.
            self._publish_gazebo(
                target_steer,
                [0.0, 0.0, 0.0] if steering_busy else wheel_speeds)
        else:
            # RViz: ramp steering explicitly in software
            for i in range(3):
                diff = target_steer[i] - self._steer[i]
                if diff:
                    self._steer[i] += math.copysign(
                        min(abs(diff), STEER_RATE * dt), diff)
            actual_speeds = [0.0, 0.0, 0.0] if steering_busy else wheel_speeds
            for i in range(3):
                self._wheel_pos[i] += actual_speeds[i] * dt
            self._publish_rviz(actual_speeds)

    def _publish_gazebo(self, steer, wheel_speeds):
        s_msg = Float64MultiArray()
        s_msg.data = list(steer)
        self.steer_pub.publish(s_msg)

        w_msg = Float64MultiArray()
        w_msg.data = [float(v) for v in wheel_speeds]
        self.wheel_pub.publish(w_msg)

    def _publish_rviz(self, wheel_speeds):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = STEER_JOINTS + WHEEL_JOINTS
        msg.position = (list(self._steer)
                        + list(self._wheel_pos))
        msg.velocity = [0.0] * 3 + [float(v) for v in wheel_speeds]
        self.js_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
