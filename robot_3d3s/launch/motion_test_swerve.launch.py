import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_3d3s')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_3d3s.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Broadcasts TF from /joint_states + URDF.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # RViz-only automated motion demo.
        # Since Gazebo / ros2_control is not running here, this node must publish
        # both /joint_states and a simple world -> base_footprint TF for RViz.
        Node(
            package='robot_3d3s',
            executable='motion_demo_swerve.py',
            name='motion_demo_swerve',
            parameters=[{
                'publish_joint_states': True,
                'publish_demo_tf': True,
                'odom_frame': 'world',
                'base_frame': 'base_footprint',
            }],
            output='screen',
        ),

        # RViz2 visualizer.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
