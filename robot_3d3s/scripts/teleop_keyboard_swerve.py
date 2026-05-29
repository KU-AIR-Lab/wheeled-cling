#!/usr/bin/env python3
"""
teleop_keyboard_swerve.py  -  Keyboard teleoperation for the 3D3S robot using swerve_controller.

Primary behavior
----------------
Publishes geometry_msgs/TwistStamped commands to:

  /swerve_controller/cmd_vel

The swerve_controller computes steering angles and wheel speeds.

Optional RViz-only visualization
--------------------------------
RViz RobotModel needs /joint_states. If running without Gazebo / ros2_control,
there is no joint_state_broadcaster, so this node can optionally publish a
simple /joint_states visualization and odom -> base_footprint TF.

Enable for RViz-only launch with:

  publish_joint_states:=true
  publish_demo_tf:=true

Do not enable these with Gazebo joint_state_broadcaster / odom_node.
"""

import math
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

PUBLISH_HZ = 50

R_WHEEL = 0.050
R_LEG = 0.38905
R_CONTACT = R_LEG + 0.025
LEG_ANGLES = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]

STEER_JOINTS = ['steering_1_joint', 'steering_2_joint', 'steering_3_joint']
WHEEL_JOINTS = ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint']

LINEAR_SPEEDS = [0.05, 0.10, 0.15, 0.20, 0.25]
ANGULAR_SPEEDS = [0.10, 0.20, 0.30, 0.50]
ZERO_SPEED_EPS = 1e-9

KEY_MAP = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
    ' ': (0, 0, 0),
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


def _wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _limit_steering_angle(angle_raw, speed_raw):
    angle = _wrap_to_pi(angle_raw)
    speed = speed_raw

    if angle > math.pi / 2.0:
        angle -= math.pi
        speed = -speed
    elif angle < -math.pi / 2.0:
        angle += math.pi
        speed = -speed

    return angle, speed


def _swerve_controller_ik(vx, vy, omega, previous_steer):
    """URDF-compatible IK matching cmd_vel_to_wheels.py / swerve_controller.

    Used only for optional RViz-only /joint_states visualization.
    In Gazebo, the real swerve_controller remains the source of truth.
    """
    steers = []
    speeds = []

    for i, alpha in enumerate(LEG_ANGLES):
        vwx = vx - math.sin(alpha) * omega * R_CONTACT
        vwy = vy + math.cos(alpha) * omega * R_CONTACT
        mag = math.hypot(vwx, vwy)

        if mag < ZERO_SPEED_EPS:
            steers.append(previous_steer[i])
            speeds.append(0.0)
            continue

        rolling_dir = math.atan2(vwy, vwx)
        phi = rolling_dir - math.pi / 2.0
        steer_raw = _wrap_to_pi(alpha - phi)
        speed_raw = mag / R_WHEEL

        steer, speed = _limit_steering_angle(steer_raw, speed_raw)
        steers.append(steer)
        speeds.append(-speed)

    return steers, speeds


def _read_key(saved_settings):
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

        self.declare_parameter('cmd_topic', '/swerve_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_joint_states', False)
        self.declare_parameter('publish_demo_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)
        self.publish_demo_tf = bool(self.get_parameter('publish_demo_tf').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.js_pub = (
            self.create_publisher(JointState, '/joint_states', 10)
            if self.publish_joint_states else None
        )
        self.tf_br = TransformBroadcaster(self) if self.publish_demo_tf else None

        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._lock = threading.Lock()

        self._lin_idx = 1
        self._ang_idx = 1

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._wheel_pos = [0.0, 0.0, 0.0]
        self._steer = [0.0, 0.0, 0.0]

        self.get_logger().info(f'teleop_keyboard_swerve ready - publishing {self.cmd_topic}')

        # Print this before the keyboard thread switches the terminal into raw
        # mode. This prevents visually shifted / indented terminal output.
        if self.publish_joint_states:
            print()
            print('Publishing /joint_states for RViz-only visualization. Disable this in Gazebo.')

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

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
        dt = 1.0 / PUBLISH_HZ
        stamp = self.get_clock().now().to_msg()

        with self._lock:
            vx = self._vx
            vy = self._vy
            omega = self._omega

        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = omega
        self.cmd_pub.publish(msg)

        if self.publish_joint_states:
            self._publish_joint_states(vx, vy, omega, dt, stamp)

        if self.publish_demo_tf:
            self._integrate_and_publish_tf(vx, vy, omega, dt, stamp)

    def _publish_joint_states(self, vx, vy, omega, dt, stamp):
        steers, wheel_vel = _swerve_controller_ik(vx, vy, omega, self._steer)
        self._steer = list(steers)

        for i in range(3):
            self._wheel_pos[i] += wheel_vel[i] * dt

        js_msg = JointState()
        js_msg.header.stamp = stamp
        js_msg.name = STEER_JOINTS + WHEEL_JOINTS
        js_msg.position = steers + self._wheel_pos
        js_msg.velocity = [0.0, 0.0, 0.0] + [float(v) for v in wheel_vel]
        self.js_pub.publish(js_msg)

    def _integrate_and_publish_tf(self, vx, vy, omega, dt, stamp):
        ct = math.cos(self._theta)
        st = math.sin(self._theta)
        self._x += (ct * vx - st * vy) * dt
        self._y += (st * vx + ct * vy) * dt
        self._theta += omega * dt

        qx, qy, qz, qw = _yaw_to_quat(self._theta)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_br.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardSwerveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = node.frame_id
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
