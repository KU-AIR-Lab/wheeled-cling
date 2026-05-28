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

        # Static world → base_footprint (robot at origin for manual GUI mode)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
            output='screen',
        ),

        # Broadcasts TF frames from joint states + URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Slider GUI: wheel speed sliders + steering angle sliders
        Node(
            package='robot_3d3s',
            executable='joint_control_gui.py',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2 visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # Passing the path; if it doesn't exist, RViz will log a warning but still open
            arguments=['-d', rviz_config],
            output='screen',        
        ),
    ])
