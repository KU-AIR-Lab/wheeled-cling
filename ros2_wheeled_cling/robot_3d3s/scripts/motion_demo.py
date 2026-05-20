#!/usr/bin/env python3
"""
motion_demo.py  —  Kinematic motion test for the 3D3S robot.

Cycles automatically through:
  Rotate CCW  →  Rotate CW  →  Forward  →  Backward  →
  Slide Left  →  Slide Right  →  (repeat)

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

Kinematics summary
------------------
Leg angles from +X (CCW): a1=0°, a2=120°, a3=240°
Wheel contact radius from robot centre: R_c = 0.414 m
Wheel radius: R_w = 0.050 m

For spin-axis angle phi_i = a_i - s_i the body-frame rolling constraint is:
  -sin(phi_i)*vx + cos(phi_i)*vy + (sin(phi_i)*py_i + cos(phi_i)*px_i)*wz
      = wheel_vel_i * R_w

Solving gives: translation v = 0.10 m/s, rotation w = 0.242 rad/s @ speed=2 rad/s.

Steering presets
  STEER_ROTATE  = [  0°,   0°,   0°]   → tangential push, pure spin
  STEER_FORWARD = [+90°, -150°, -30°]  → all spin-axes at 270° → +X
  STEER_LATERAL = [  0°, +120°, -120°] → all spin-axes at   0° → +Y
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
R_CONTACT = R_LEG + 0.025  # wheel contact point radius (m) = 0.41405

LEG_ANGLES = [0.0, 2*math.pi/3, 4*math.pi/3]   # a1, a2, a3 in rad

# Contact point (x, y) in body frame for each wheel
P_CONTACT = [
    ( R_CONTACT * math.cos(a),  R_CONTACT * math.sin(a))
    for a in LEG_ANGLES
]

# ── Kinematic constants ───────────────────────────────────────────────────────

DRIVE_SPEED = 2.0    # rad/s applied to all wheels during driving
STEER_RATE  = 1.2    # rad/s steering transition rate
PUBLISH_HZ  = 50

# Steering angle sets  [s_1, s_2, s_3]
STEER_ROTATE  = [0.0,              0.0,             0.0           ]
STEER_FORWARD = [math.pi/2,       -5*math.pi/6,    -math.pi/6    ]
STEER_LATERAL = [0.0,              2*math.pi/3,    -2*math.pi/3   ]

# Motion sequence: (label, target_steering, wheel_sign, drive_sec)
# wheel_sign is negative for "positive" motion because rolling-forward requires
# the contact patch to move backward → negative joint angle rate (right-hand rule
# around x_wheel with y_wheel=UP and z_wheel=rolling-forward gives this convention).
MOTION_SEQUENCE = [
    ('Rotate CCW',     STEER_ROTATE,  -1, 3.0),
    ('Rotate CW',      STEER_ROTATE,  +1, 3.0),
    ('Forward  (+X)',  STEER_FORWARD, -1, 3.0),
    ('Backward (-X)',  STEER_FORWARD, +1, 3.0),
    ('Slide Left (+Y)',STEER_LATERAL, -1, 3.0),
    ('Slide Right(-Y)',STEER_LATERAL, +1, 3.0),
]

STEER_PAUSE  = 0.5   # seconds between DRIVE and next STEER phase
STEER_MAXSEC = 3.5   # cap on steering transition time


# ── Dead-reckoning helpers ────────────────────────────────────────────────────

def _body_velocity(steer, wheel_vel):
    """Return [vx, vy, omega_z] in the robot body frame.

    No-slip rolling constraint (derived via v_wheel_center + ω×r_contact = 0):
      A(steer) · [vx, vy, wz] = -wheel_vel_i * R_WHEEL
    Solved with numpy least-squares (robust near-singular during transitions).
    """
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for i, (a, s, wv, (px, py)) in enumerate(
            zip(LEG_ANGLES, steer, wheel_vel, P_CONTACT)):
        phi = a - s                          # spin-axis angle in world(-body) frame
        sp, cp = math.sin(phi), math.cos(phi)
        A[i] = [-sp, cp, sp*py + cp*px]
        b[i] = -wv * R_WHEEL   # no-slip: A·v_body = -ω_wheel × R_wheel
    v, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return v  # [vx, vy, wz]


def _yaw_to_quat(yaw):
    """Return (qx, qy, qz, qw) for a pure yaw rotation."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


# ── Node ─────────────────────────────────────────────────────────────────────

class MotionDemoNode(Node):

    def __init__(self):
        super().__init__('motion_demo')

        self.declare_parameter('gazebo', False)
        self.gazebo_mode = self.get_parameter('gazebo').value

        # Publishers
        if self.gazebo_mode:
            self.steer_pub = self.create_publisher(
                Float64MultiArray, '/steering_controller/commands', 10)
            self.wheel_pub = self.create_publisher(
                Float64MultiArray, '/wheel_controller/commands', 10)
            self.get_logger().info(
                'Gazebo mode — '
                '/steering_controller/commands + /wheel_controller/commands')
        else:
            self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.get_logger().info('RViz mode — /joint_states')

        # TF broadcaster (world → base_footprint) for both modes
        self._tf_br = TransformBroadcaster(self)

        # Robot state (protected by _lock)
        self._steer     = [0.0, 0.0, 0.0]
        self._wheel_pos = [0.0, 0.0, 0.0]   # integrated for RViz animation
        self._wheel_vel = [0.0, 0.0, 0.0]

        # Dead-reckoning pose in world frame
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0

        self._target_steer = [0.0, 0.0, 0.0]
        self._lock = threading.Lock()

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        threading.Thread(target=self._planner, daemon=True).start()

    # ── Planner thread ────────────────────────────────────────────────────────

    def _planner(self):
        import time
        time.sleep(1.0)

        step = 0
        while rclpy.ok():
            label, target_s, wsign, drive_sec = MOTION_SEQUENCE[step]

            # Steering transition (wheels stopped)
            with self._lock:
                self._wheel_vel  = [0.0, 0.0, 0.0]
                self._target_steer = list(target_s)

            delta = max(abs(target_s[i] - self._steer[i]) for i in range(3))
            steer_sec = min(delta / STEER_RATE + 0.3, STEER_MAXSEC)

            deg = [math.degrees(s) for s in target_s]
            self.get_logger().info(
                f'\n{"─"*60}\n'
                f'  NEXT: {label}\n'
                f'  Steer → [{deg[0]:+.1f}°, {deg[1]:+.1f}°, {deg[2]:+.1f}°]\n'
                f'{"─"*60}')
            time.sleep(steer_sec)

            # Drive
            with self._lock:
                self._wheel_vel = [wsign * DRIVE_SPEED] * 3
            self.get_logger().info(
                f'  DRIVE  {drive_sec:.1f}s  '
                f'({"+" if wsign > 0 else "-"}{DRIVE_SPEED} rad/s)')
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
            self._steer[i] += math.copysign(
                min(abs(diff), STEER_RATE * dt), diff)

        # Integrate wheel position for RViz animation
        for i in range(3):
            self._wheel_pos[i] += wheel_vel[i] * dt

        # Dead-reckoning: compute body velocity → integrate world pose
        v = _body_velocity(self._steer, wheel_vel)  # [vx, vy, wz] in body frame
        ct, st = math.cos(self._theta), math.sin(self._theta)
        self._x     += (ct * v[0] - st * v[1]) * dt
        self._y     += (st * v[0] + ct * v[1]) * dt
        self._theta += v[2] * dt

        # Publish TF: world → base_footprint
        self._publish_tf()

        # Publish joint commands
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
