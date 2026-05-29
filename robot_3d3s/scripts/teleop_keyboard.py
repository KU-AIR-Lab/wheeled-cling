#!/usr/bin/env python3
"""
teleop_keyboard.py  —  Keyboard teleoperation for the 3D3S robot.

Two behaviours built on top of the inverse kinematics
------------------------------------------------------
1. Steering-first gate
   Whenever a new motion command requires steering to move more than
   STEER_THRESHOLD (5°), the wheels are held at zero until the steering
   reaches the target.  Only then do the wheels start spinning.
   This prevents the robot from skidding while the axles re-orient.

2. Minimal-steering / smart-flip
   For each wheel the IK produces two physically equivalent solutions:
     (a)  angle      with speed  w
     (b)  angle ± π  with speed -w
   The solution whose angle is closest to the CURRENT steering position
   is chosen.  Going from Forward → Backward keeps the steering exactly
   where it is and simply reverses the wheel — no steering movement at all.

Controls
--------
  W / S       Forward / Backward      (+X / -X body frame)
  A / D       Slide Left / Right      (+Y / -Y body frame)
  Q / E       Rotate CCW / CW
  Space       STOP
  [ / ]       Decrease / Increase linear speed
  { / }       Decrease / Increase angular speed
  Ctrl+C      Quit

Two modes (same as motion_demo.py)
-----------------------------------
RViz mode (default):
  ros2 launch robot_3d3s teleop.launch.py

Gazebo mode:
  Terminal 1:  ros2 launch robot_3d3s gazebo.launch.py
  Terminal 2:  ros2 run robot_3d3s teleop_keyboard.py --ros-args -p gazebo:=true
"""

import math
import sys
import tty
import termios
import select
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ── Robot geometry (must match motion_demo.py) ────────────────────────────────

R_WHEEL   = 0.050
R_LEG     = 0.38905
R_CONTACT = R_LEG + 0.025

LEG_ANGLES = [0.0, 2*math.pi/3, 4*math.pi/3]

P_CONTACT = [
    (R_CONTACT * math.cos(a), R_CONTACT * math.sin(a))
    for a in LEG_ANGLES
]

# ── Control parameters ────────────────────────────────────────────────────────

STEER_RATE      = 1.2              # rad/s — how fast steering ramps
STEER_THRESHOLD = math.radians(5)  # wheels wait until all steering errors < 5°
PUBLISH_HZ      = 50

LINEAR_SPEEDS  = [0.05, 0.10, 0.15, 0.20, 0.25]
ANGULAR_SPEEDS = [0.10, 0.20, 0.30, 0.50]

# ── Key map ───────────────────────────────────────────────────────────────────

KEY_MAP = {
    'w': ( 1,  0,  0),   # forward  (+X)
    's': (-1,  0,  0),   # backward (-X)
    'a': ( 0,  1,  0),   # slide left  (+Y)
    'd': ( 0, -1,  0),   # slide right (-Y)
    'q': ( 0,  0,  1),   # rotate CCW
    'e': ( 0,  0, -1),   # rotate CW
    ' ': ( 0,  0,  0),   # stop
}

KEY_LABELS = {
    'w': 'Forward', 's': 'Backward',
    'a': 'Slide Left', 'd': 'Slide Right',
    'q': 'Rotate CCW', 'e': 'Rotate CW',
    ' ': 'STOP',
}

HELP = """
──────────────────────────────────────────────────
  3D3S Robot  ·  Keyboard Teleop
──────────────────────────────────────────────────
  W / S   : Forward  / Backward
  A / D   : Slide Left / Slide Right
  Q / E   : Rotate CCW / Rotate CW
  Space   : STOP
  [ / ]   : Linear speed  ─ / +
  { / }   : Angular speed ─ / +
  Ctrl+C  : Quit
──────────────────────────────────────────────────
  Logic: steering moves first, then wheels spin.
  Forward → Backward: wheels reverse, NO re-steering.
──────────────────────────────────────────────────
"""


# ── Kinematics ────────────────────────────────────────────────────────────────

def _ang_dist(a, b):
    """Shortest angular distance between two angles (always in [0, π])."""
    d = abs(a - b) % (2 * math.pi)
    return min(d, 2 * math.pi - d)


def _inverse_kinematics(vx, vy, omega):
    """Raw IK: returns (steering_angles, wheel_speeds) without considering
    current state.  Steering angles are limited to [-π/2, π/2] by flipping
    wheel direction — matching the advisor's MATLAB limitSteeringAngle."""
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
        s   = (s + math.pi) % (2 * math.pi) - math.pi   # wrap to (−π, π]

        sign = 1.0
        if s > math.pi / 2:
            s    -= math.pi
            sign  = -1.0
        elif s < -math.pi / 2:
            s    += math.pi
            sign  = -1.0

        steers.append(s)
        speeds.append(-sign * mag / R_WHEEL)
    return steers, speeds


def _smart_commands(vx, vy, omega, current_steer):
    """IK with minimal-steering optimisation.

    For each wheel, compare the raw IK solution with the flipped alternative
    (angle ± π, speed negated).  Choose whichever steering angle is closer to
    the CURRENT position, avoiding unnecessary 180° re-steers.

    When velocity is zero the current steering is preserved and wheels stop.
    """
    if abs(vx) + abs(vy) + abs(omega) < 1e-9:
        return list(current_steer), [0.0, 0.0, 0.0]

    raw_steers, raw_speeds = _inverse_kinematics(vx, vy, omega)

    steers, speeds = [], []
    for s_new, w_new, s_cur in zip(raw_steers, raw_speeds, current_steer):
        if abs(w_new) < 1e-9:
            steers.append(s_cur)   # keep current angle when this wheel is stopped
            speeds.append(0.0)
            continue

        # Flipped alternative: ±π shift keeps the wheel pointing the same way
        # physically, but the sign of the speed is reversed.
        s_flip = s_new - math.pi if s_new > 0 else s_new + math.pi

        if _ang_dist(s_flip, s_cur) < _ang_dist(s_new, s_cur):
            steers.append(s_flip)
            speeds.append(-w_new)
        else:
            steers.append(s_new)
            speeds.append(w_new)

    return steers, speeds


def _body_velocity(steer, wheel_vel):
    """Forward kinematics for dead-reckoning: [vx, vy, ωz] in body frame."""
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


# ── Keyboard helper ───────────────────────────────────────────────────────────

def _read_key(saved_settings):
    """Return next keypress or '' on 0.1 s timeout."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if ready else ''
    if key == '\x1b' and ready:
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_settings)
    return key


# ── Node ─────────────────────────────────────────────────────────────────────

class TeleopKeyboardNode(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')

        self.declare_parameter('gazebo', False)
        self.gazebo_mode = self.get_parameter('gazebo').value

        if self.gazebo_mode:
            self.steer_pub = self.create_publisher(
                Float64MultiArray, '/steering_controller/commands', 10)
            self.wheel_pub = self.create_publisher(
                Float64MultiArray, '/wheel_controller/commands', 10)
            self.get_logger().info('Gazebo mode')
        else:
            self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.get_logger().info('RViz mode')

        self._tf_br = TransformBroadcaster(self)

        # Desired body velocity — written by keyboard thread, read by timer
        self._vx    = 0.0
        self._vy    = 0.0
        self._omega = 0.0
        self._lock  = threading.Lock()

        # Ramped steering state (body frame)
        self._steer     = [0.0, 0.0, 0.0]
        self._wheel_pos = [0.0, 0.0, 0.0]   # integrated for RViz

        # Dead-reckoning world pose
        self._x = self._y = self._theta = 0.0

        self._lin_idx = 1   # default 0.10 m/s
        self._ang_idx = 1   # default 0.20 rad/s

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

    # ── Properties ───────────────────────────────────────────────────────────

    @property
    def _lin_speed(self):
        return LINEAR_SPEEDS[self._lin_idx]

    @property
    def _ang_speed(self):
        return ANGULAR_SPEEDS[self._ang_idx]

    # ── Keyboard loop ─────────────────────────────────────────────────────────

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
                        self._vx    = lx * self._lin_speed
                        self._vy    = ly * self._lin_speed
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
                self._vx = self._vy = self._omega = 0.0

    def _print_state(self, key):
        with self._lock:
            vx, vy, omega = self._vx, self._vy, self._omega
        label = KEY_LABELS.get(key, key)
        print(f'  [{label:12s}]  Vx={vx:+.2f} m/s  Vy={vy:+.2f} m/s  Ω={omega:+.3f} rad/s')

    # ── 50 Hz publish callback ────────────────────────────────────────────────

    def _publish_cb(self):
        dt = 1.0 / PUBLISH_HZ
        with self._lock:
            vx, vy, omega = self._vx, self._vy, self._omega

        # Smart IK: choose steering solution closest to current position
        target_steer, wheel_speeds = _smart_commands(vx, vy, omega, self._steer)

        # ── Steering-first gate ───────────────────────────────────────────────
        # Find the largest steering error across all three wheels.
        max_err = max(
            _ang_dist(target_steer[i], self._steer[i]) for i in range(3)
        )
        steering_busy = max_err > STEER_THRESHOLD

        # Ramp each steering joint toward its target
        for i in range(3):
            diff = target_steer[i] - self._steer[i]
            if diff:
                self._steer[i] += math.copysign(
                    min(abs(diff), STEER_RATE * dt), diff)

        # Hold wheels at zero while any steering is still moving
        actual_speeds = [0.0, 0.0, 0.0] if steering_busy else wheel_speeds

        # ── Integrate wheel angle (RViz animation) ───────────────────────────
        for i in range(3):
            self._wheel_pos[i] += actual_speeds[i] * dt

        # ── Dead-reckoning world pose ─────────────────────────────────────────
        v  = _body_velocity(self._steer, actual_speeds)
        ct = math.cos(self._theta)
        st = math.sin(self._theta)
        self._x     += (ct * v[0] - st * v[1]) * dt
        self._y     += (st * v[0] + ct * v[1]) * dt
        self._theta += v[2] * dt

        self._publish_tf()

        if self.gazebo_mode:
            self._publish_gazebo(actual_speeds)
        else:
            self._publish_rviz(actual_speeds)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = _yaw_to_quat(self._theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_br.sendTransform(t)

    def _publish_gazebo(self, wheel_speeds):
        s_msg = Float64MultiArray()
        s_msg.data = list(self._steer)
        self.steer_pub.publish(s_msg)

        w_msg = Float64MultiArray()
        w_msg.data = [float(v) for v in wheel_speeds]
        self.wheel_pub.publish(w_msg)

    def _publish_rviz(self, wheel_speeds):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'steering_1_joint', 'steering_2_joint', 'steering_3_joint',
            'wheel_1_joint',    'wheel_2_joint',    'wheel_3_joint',
        ]
        msg.position = [
            self._steer[0],     self._steer[1],     self._steer[2],
            self._wheel_pos[0], self._wheel_pos[1], self._wheel_pos[2],
        ]
        msg.velocity = [0.0] * 3 + [float(v) for v in wheel_speeds]
        self.js_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
