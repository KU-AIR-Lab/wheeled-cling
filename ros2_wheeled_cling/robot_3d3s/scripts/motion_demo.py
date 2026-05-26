#!/usr/bin/env python3
"""
motion_demo.py  —  Kinematic motion test for the 3D3S robot.

Cycles automatically through:
  Rotate CCW  →  Rotate CW  →  Forward  →  Backward  →
  Slide Left  →  Slide Right  →  (repeat)

Steering angles are guaranteed to stay within [-90°, +90°] at all times.
When the IK solution would exceed this range, the wheel direction is flipped
instead (same physical motion, no collision risk).

Two modes
---------
RViz mode (default):
  Publishes /joint_states AND a TF transform world → base_footprint based on
  dead-reckoning.  Set RViz Fixed Frame to "world" to watch the robot translate
  and rotate on the grid.

Gazebo mode  (pass --ros-args -p gazebo:=true):
  Publishes to /steering_controller/commands (Float64MultiArray, positions)
  and /wheel_controller/commands (Float64MultiArray, velocities).
  Dead-reckoning TF is still published so RViz can track the robot alongside.
  Requires gazebo.launch.py to be running.

Usage
-----
RViz only:
  ros2 launch robot_3d3s motion_test.launch.py

Gazebo:
  Terminal 1:  ros2 launch robot_3d3s gazebo.launch.py
  Terminal 2:  ros2 run robot_3d3s motion_demo.py --ros-args -p gazebo:=true
"""

import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ── Robot geometry ────────────────────────────────────────────────────────────

R_WHEEL   = 0.050          # wheel radius (m)
R_LEG     = 0.38905        # steering-pivot radius from robot centre (m)
R_CONTACT = R_LEG + 0.025  # wheel contact point radius (m)

LEG_ANGLES = [0.0, 2*math.pi/3, 4*math.pi/3]

P_CONTACT = [
    (R_CONTACT * math.cos(a), R_CONTACT * math.sin(a))
    for a in LEG_ANGLES
]

# ── Kinematic constants ───────────────────────────────────────────────────────

DRIVE_SPEED = 2.0   # rad/s wheel speed magnitude for all motions
STEER_RATE  = 1.2   # rad/s steering ramp speed
PUBLISH_HZ  = 50

# Velocity commands derived from DRIVE_SPEED so all motions use equal wheel effort
V_LINEAR  = DRIVE_SPEED * R_WHEEL             # 0.10 m/s  → DRIVE_SPEED rad/s at wheel
V_ANGULAR = DRIVE_SPEED * R_WHEEL / R_CONTACT  # 0.242 rad/s → DRIVE_SPEED rad/s at wheel

# Motion sequence: (label, vx, vy, omega, drive_seconds)
# Steering angles are computed via IK — always in [-90°, +90°].
MOTION_SEQUENCE = [
    ('Rotate CCW',       0.0,       0.0,      +V_ANGULAR, 3.0),
    ('Rotate CW',        0.0,       0.0,      -V_ANGULAR, 3.0),
    ('Forward  (+X)',   +V_LINEAR,  0.0,       0.0,       3.0),
    ('Backward (-X)',   -V_LINEAR,  0.0,       0.0,       3.0),
    ('Slide Left (+Y)', 0.0,       +V_LINEAR,  0.0,       3.0),
    ('Slide Right(-Y)', 0.0,       -V_LINEAR,  0.0,       3.0),
]

STEER_PAUSE  = 0.5   # seconds pause between drive and next steer phase
STEER_MAXSEC = 3.5   # cap on steering transition time


# ── Inverse kinematics ────────────────────────────────────────────────────────

def _inverse_kinematics(vx, vy, omega):
    """Compute (steer_angles, wheel_speeds) from body velocity.

    Steering angles are clipped to [-π/2, π/2] by flipping wheel direction,
    so mechanical collision from over-rotation is impossible.
    """
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
        s   = (s + math.pi) % (2 * math.pi) - math.pi   # wrap to [-π, π]
        sign = 1.0
        if s > math.pi / 2:      # clip to [-90°, +90°] by flipping wheel
            s -= math.pi; sign = -1.0
        elif s < -math.pi / 2:
            s += math.pi; sign = -1.0
        steers.append(s)
        speeds.append(-sign * mag / R_WHEEL)
    return steers, speeds


# ── Dead-reckoning helpers ────────────────────────────────────────────────────

def _body_velocity(steer, wheel_vel):
    """Return [vx, vy, omega_z] in robot body frame from joint states."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for i, (a, s, wv, (px, py)) in enumerate(
            zip(LEG_ANGLES, steer, wheel_vel, P_CONTACT)):
        phi = a - s
        sp, cp = math.sin(phi), math.cos(phi)
        A[i] = [-sp, cp, sp*py + cp*px]
        b[i] = -wv * R_WHEEL
    v, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return v


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


# ── Node ─────────────────────────────────────────────────────────────────────

class MotionDemoNode(Node):

    def __init__(self):
        super().__init__('motion_demo')

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

        self._steer     = [0.0, 0.0, 0.0]
        self._wheel_pos = [0.0, 0.0, 0.0]
        self._wheel_vel = [0.0, 0.0, 0.0]
        self._target_steer = [0.0, 0.0, 0.0]

        self._x = self._y = self._theta = 0.0
        self._lock = threading.Lock()

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        threading.Thread(target=self._planner, daemon=True).start()

    # ── Planner thread ────────────────────────────────────────────────────────

    def _planner(self):
        import time
        time.sleep(1.0)

        step = 0
        while rclpy.ok():
            label, vx, vy, omega, drive_sec = MOTION_SEQUENCE[step]

            # Compute IK — steering guaranteed within [-90°, +90°]
            target_steer, target_speeds = _inverse_kinematics(vx, vy, omega)

            # Stop wheels, set new steering target
            with self._lock:
                self._wheel_vel    = [0.0, 0.0, 0.0]
                self._target_steer = target_steer

            delta = max(abs(target_steer[i] - self._steer[i]) for i in range(3))
            steer_sec = min(delta / STEER_RATE + 0.3, STEER_MAXSEC)

            deg = [math.degrees(s) for s in target_steer]
            spd = [f'{v:+.2f}' for v in target_speeds]
            self.get_logger().info(
                f'\n{"─"*60}\n'
                f'  NEXT : {label}\n'
                f'  Steer: [{deg[0]:+.1f}°, {deg[1]:+.1f}°, {deg[2]:+.1f}°]'
                f'  (all within ±90°)\n'
                f'  Speed: [{spd[0]}, {spd[1]}, {spd[2]}] rad/s\n'
                f'{"─"*60}')
            time.sleep(steer_sec)

            # Drive
            with self._lock:
                self._wheel_vel = list(target_speeds)
            self.get_logger().info(f'  DRIVE  {drive_sec:.1f}s')
            time.sleep(drive_sec)

            # Stop
            with self._lock:
                self._wheel_vel = [0.0, 0.0, 0.0]
            time.sleep(STEER_PAUSE)

            step = (step + 1) % len(MOTION_SEQUENCE)

    # ── Publish callback (50 Hz) ──────────────────────────────────────────────

    def _publish_cb(self):
        dt = 1.0 / PUBLISH_HZ
        with self._lock:
            target_s  = list(self._target_steer)
            wheel_vel = list(self._wheel_vel)

        # Ramp steering toward target
        for i in range(3):
            diff = target_s[i] - self._steer[i]
            self._steer[i] += math.copysign(min(abs(diff), STEER_RATE * dt), diff)

        # Integrate wheel positions for RViz animation
        for i in range(3):
            self._wheel_pos[i] += wheel_vel[i] * dt

        # Dead-reckoning pose
        v = _body_velocity(self._steer, wheel_vel)
        ct, st = math.cos(self._theta), math.sin(self._theta)
        self._x     += (ct * v[0] - st * v[1]) * dt
        self._y     += (st * v[0] + ct * v[1]) * dt
        self._theta += v[2] * dt

        self._publish_tf()
        if self.gazebo_mode:
            self._publish_gazebo(wheel_vel)
        else:
            self._publish_rviz(wheel_vel)

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

    def _publish_gazebo(self, wheel_vel):
        s_msg = Float64MultiArray()
        s_msg.data = list(self._steer)
        self.steer_pub.publish(s_msg)

        w_msg = Float64MultiArray()
        w_msg.data = [float(v) for v in wheel_vel]
        self.wheel_pub.publish(w_msg)

    def _publish_rviz(self, wheel_vel):
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
        msg.velocity = [0.0]*3 + [float(v) for v in wheel_vel]
        self.js_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MotionDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
