#!/usr/bin/env python3
"""
motion_demo_swerve.py  -  Motion command demo for the 3D3S swerve controller.

Primary behavior
----------------
This node publishes geometry_msgs/TwistStamped commands directly to:

  /swerve_controller/cmd_vel

The swerve_controller computes the steering angles and wheel velocities.

Optional RViz-only visualization
--------------------------------
RViz RobotModel needs /joint_states. If you run without Gazebo / ros2_control,
there is no joint_state_broadcaster, so this node can optionally publish a
simple /joint_states visualization using the same URDF-compatible kinematic
convention used by the swerve_controller.

Set:

  publish_joint_states:=true

for RViz-only use.

Usage
-----
Gazebo / ros2_control swerve controller:
  Terminal 1: ros2 launch robot_3d3s gazebo_swerve.launch.py
  Terminal 2: ros2 run robot_3d3s motion_demo_swerve.py

RViz-only visualization:
  ros2 run robot_3d3s motion_demo_swerve.py --ros-args -p publish_joint_states:=true -p publish_demo_tf:=true

You can inspect commands computed by the controller with:
  ros2 topic echo /swerve_controller/swerve_cmd
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# Robot geometry.
R_WHEEL = 0.050
R_LEG = 0.38905
R_CONTACT = R_LEG + 0.025

LEG_ANGLES = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]

STEER_JOINTS = ['steering_1_joint', 'steering_2_joint', 'steering_3_joint']
WHEEL_JOINTS = ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint']

# Demo constants.
DRIVE_SPEED = 2.0
PUBLISH_HZ = 50

# Velocity commands derived from DRIVE_SPEED so translational and rotational
# demos use comparable wheel effort.
V_LINEAR = DRIVE_SPEED * R_WHEEL
V_ANGULAR = DRIVE_SPEED * R_WHEEL / R_CONTACT

# Motion sequence: (label, vx, vy, omega, seconds)
# Body-frame convention: x/y/z are interpreted by the active swerve_controller.
MOTION_SEQUENCE = [
    ('Rotate CCW',        0.0,       0.0,      +V_ANGULAR, 10.0),
    ('Rotate CW',         0.0,       0.0,      -V_ANGULAR, 10.0),
    ('Forward  (+X)',    +V_LINEAR,  0.0,       0.0,       10.0),
    ('Backward (-X)',    -V_LINEAR,  0.0,       0.0,       10.0),
    ('Slide Left (+Y)',   0.0,      +V_LINEAR,  0.0,       10.0),
    ('Slide Right(-Y)',   0.0,      -V_LINEAR,  0.0,       10.0),
]

STOP_PAUSE = 0.5
ZERO_SPEED_EPS = 1e-9


def _wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _limit_steering_angle(angle_raw, speed_raw):
    """Limit steering to [-pi/2, pi/2] and flip wheel speed if needed."""
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

    This is used only for optional RViz-only /joint_states visualization.
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


class MotionDemoSwerveNode(Node):
    def __init__(self):
        super().__init__('motion_demo_swerve')

        self.declare_parameter('cmd_topic', '/swerve_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_demo_tf', False)
        self.declare_parameter('publish_joint_states', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_demo_tf = bool(self.get_parameter('publish_demo_tf').value)
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)
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

        # Optional RViz-only state. Disabled by default to avoid conflicting
        # with real joint_state_broadcaster / odometry sources.
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._steer = [0.0, 0.0, 0.0]
        self._wheel_pos = [0.0, 0.0, 0.0]

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        threading.Thread(target=self._planner, daemon=True).start()

        self.get_logger().info(f'Publishing TwistStamped commands to {self.cmd_topic}')
        self.get_logger().info('The swerve_controller is responsible for inverse kinematics in Gazebo.')
        if self.publish_joint_states:
            self.get_logger().warn(
                'Publishing /joint_states for RViz-only visualization. Do not enable this with Gazebo joint_state_broadcaster.'
            )

    def _planner(self):
        time.sleep(1.0)

        step = 0
        while rclpy.ok():
            label, vx, vy, omega, drive_sec = MOTION_SEQUENCE[step]

            with self._lock:
                self._vx = vx
                self._vy = vy
                self._omega = omega

            self.get_logger().info(
                f'NEXT: {label} | vx={vx:+.3f} m/s, '
                f'vy={vy:+.3f} m/s, omega={omega:+.3f} rad/s')
            time.sleep(drive_sec)

            with self._lock:
                self._vx = 0.0
                self._vy = 0.0
                self._omega = 0.0

            self.get_logger().info(f'STOP {STOP_PAUSE:.1f}s')
            time.sleep(STOP_PAUSE)

            step = (step + 1) % len(MOTION_SEQUENCE)

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
    node = MotionDemoSwerveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send one explicit stop before exiting.
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = node.frame_id
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
