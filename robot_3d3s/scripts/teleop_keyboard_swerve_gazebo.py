#!/usr/bin/env python3
"""
teleop_keyboard_swerve.py  -  Keyboard teleoperation for the 3D3S robot using swerve_controller.

This node does not compute inverse kinematics and does not publish directly to
separate steering / wheel controllers. Instead, it publishes body velocity
commands as geometry_msgs/TwistStamped to:

  /swerve_controller/cmd_vel

The swerve_controller receives the TwistStamped command and computes the
steering angles and wheel speeds internally.

Controls
--------
  W / S       Forward / Backward      (+X / -X body frame)
  A / D       Slide Left / Right      (+Y / -Y body frame)
  Q / E       Rotate CCW / CW
  Space       STOP
  [ / ]       Decrease / Increase linear speed
  { / }       Decrease / Increase angular speed
  Ctrl+C      Quit

Usage
-----
  Terminal 1: ros2 launch robot_3d3s gazebo_swerve.launch.py
  Terminal 2: ros2 run robot_3d3s teleop_keyboard_swerve.py

The swerve_controller YAML should use:

  use_stamped_vel: true
"""

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

PUBLISH_HZ = 50

LINEAR_SPEEDS = [0.05, 0.10, 0.15, 0.20, 0.25]
ANGULAR_SPEEDS = [0.10, 0.20, 0.30, 0.50]

KEY_MAP = {
    'w': (1, 0, 0),    # forward  (+X)
    's': (-1, 0, 0),   # backward (-X)
    'a': (0, 1, 0),    # slide left  (+Y)
    'd': (0, -1, 0),   # slide right (-Y)
    'q': (0, 0, 1),    # rotate CCW
    'e': (0, 0, -1),   # rotate CW
    ' ': (0, 0, 0),    # stop
}

KEY_LABELS = {
    'w': 'Forward',
    's': 'Backward',
    'a': 'Slide Left',
    'd': 'Slide Right',
    'q': 'Rotate CCW',
    'e': 'Rotate CW',
    ' ': 'STOP',
}

HELP = """
--------------------------------------------------
  3D3S Robot  -  Swerve Controller Keyboard Teleop
--------------------------------------------------
  W / S   : Forward / Backward
  A / D   : Slide Left / Slide Right
  Q / E   : Rotate CCW / Rotate CW
  Space   : STOP
  [ / ]   : Linear speed  - / +
  { / }   : Angular speed - / +
  Ctrl+C  : Quit
--------------------------------------------------
  Publishes TwistStamped to /swerve_controller/cmd_vel
  The swerve_controller computes steering and wheel commands.
--------------------------------------------------
"""


def _read_key(saved_settings):
    """Return next keypress or empty string on timeout."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if ready else ''
    if key == '\x1b' and ready:
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_settings)
    return key


class TeleopKeyboardSwerveNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_swerve')

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/swerve_controller/cmd_vel',
            10,
        )

        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._lock = threading.Lock()

        self._lin_idx = 1
        self._ang_idx = 1

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info('teleop_keyboard_swerve ready - publishing /swerve_controller/cmd_vel')

    @property
    def _lin_speed(self):
        return LINEAR_SPEEDS[self._lin_idx]

    @property
    def _ang_speed(self):
        return ANGULAR_SPEEDS[self._ang_idx]

    def _keyboard_loop(self):
        saved = termios.tcgetattr(sys.stdin)
        print(HELP)
        print(f'  Linear speed : {self._lin_speed:.2f} m/s')
        print(f'  Angular speed: {self._ang_speed:.2f} rad/s\n')

        try:
            while rclpy.ok():
                key = _read_key(saved)
                if not key:
                    continue
                if key == '\x03':
                    break

                if key in KEY_MAP:
                    lx, ly, lw = KEY_MAP[key]
                    with self._lock:
                        self._vx = lx * self._lin_speed
                        self._vy = ly * self._lin_speed
                        self._omega = lw * self._ang_speed
                    self._print_state(key)

                elif key == '[':
                    self._lin_idx = max(0, self._lin_idx - 1)
                    print(f'  Linear speed : {self._lin_speed:.2f} m/s')
                elif key == ']':
                    self._lin_idx = min(len(LINEAR_SPEEDS) - 1, self._lin_idx + 1)
                    print(f'  Linear speed : {self._lin_speed:.2f} m/s')
                elif key == '{':
                    self._ang_idx = max(0, self._ang_idx - 1)
                    print(f'  Angular speed: {self._ang_speed:.2f} rad/s')
                elif key == '}':
                    self._ang_idx = min(len(ANGULAR_SPEEDS) - 1, self._ang_idx + 1)
                    print(f'  Angular speed: {self._ang_speed:.2f} rad/s')

        except Exception as exc:
            self.get_logger().error(f'Keyboard thread: {exc}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
            with self._lock:
                self._vx = 0.0
                self._vy = 0.0
                self._omega = 0.0

    def _print_state(self, key):
        with self._lock:
            vx = self._vx
            vy = self._vy
            omega = self._omega
        label = KEY_LABELS.get(key, key)
        print(f'  [{label:12s}]  Vx={vx:+.2f} m/s  Vy={vy:+.2f} m/s  Omega={omega:+.3f} rad/s')

    def _publish_cb(self):
        with self._lock:
            vx = self._vx
            vy = self._vy
            omega = self._omega

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = omega

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardSwerveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
