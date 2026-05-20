import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_3d3s')
    urdf_file  = os.path.join(pkg_share, 'urdf', 'robot_3d3s.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Broadcasts TF from joint states + URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Automated motion-demo node (RViz mode — publishes /joint_states)
        Node(
            package='robot_3d3s',
            executable='motion_demo.py',
            name='motion_demo',
            output='screen',
        ),

        # RViz2 visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
