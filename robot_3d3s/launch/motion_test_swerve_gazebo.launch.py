import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_3d3s')

    gazebo_swerve_launch = os.path.join(
        pkg_share,
        'launch',
        'gazebo_swerve.launch.py'
    )

    rviz_config = os.path.join(
        pkg_share,
        'config',
        'display.rviz'
    )

    gazebo_swerve = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_swerve_launch)
    )

    # Gazebo provides /joint_states through joint_state_broadcaster, but RViz
    # also needs a dynamic odom -> base_footprint transform. This node consumes
    # /joint_states and publishes /odom plus TF odom -> base_footprint.
    odom_node = Node(
        package='robot_3d3s',
        executable='odom_node.py',
        name='odom_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Start after gazebo_swerve.launch.py has spawned the robot and activated
    # joint_state_broadcaster + swerve_controller.
    motion_demo_swerve = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='robot_3d3s',
                executable='motion_demo_swerve.py',
                name='motion_demo_swerve',
                output='screen',
                parameters=[{
                    # Gazebo / ros2_control is the source of joint states and odom.
                    # Keep these false to avoid duplicate /joint_states or duplicate TF.
                    'publish_joint_states': False,
                    'publish_demo_tf': False,
                    'use_sim_time': True,
                }],
            )
        ],
    )

    # Delay RViz until robot_state_publisher, joint_state_broadcaster, and odom_node
    # have had time to publish the TF chain world -> odom -> base_footprint -> base_link.
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        gazebo_swerve,
        odom_node,
        rviz,
        motion_demo_swerve,
    ])
