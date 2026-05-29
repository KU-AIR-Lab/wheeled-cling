import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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

        # Run teleop in a real terminal so it can capture keyboard input.
        # RViz-only mode needs /joint_states and a TF chain to the RViz fixed
        # frame. display.rviz uses world, so publish world -> base_footprint.
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'bash', '-lc',
                'ros2 run robot_3d3s teleop_keyboard_swerve.py '
                '--ros-args '
                '-p publish_joint_states:=true '
                '-p publish_demo_tf:=true '
                '-p odom_frame:=world '
                '-p base_frame:=base_footprint; '
                'exec bash'
            ],
            output='screen',
        ),

        # Start RViz slightly after the publishers to avoid initial TF warnings.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                )
            ],
        ),
    ])
