import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file       = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    world_path      = os.path.join(pkg, 'worlds', 'my_world.sdf')
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')

    # Read URDF and inject the absolute path to controllers.yaml into the
    # gz_ros2_control plugin tag (replaces the CONTROLLERS_YAML_PATH placeholder).
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('CONTROLLERS_YAML_PATH', controllers_yaml)

    # 1. Gazebo Sim with custom world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
    )

    # 2. Spawn the robot (with injected URDF so Gazebo sees the correct plugin path)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',   'robot_3d3s',
            '-string', robot_desc,   # pass modified URDF as a string
            '-x', '0', '-y', '0', '-z', '0.2',
        ],
        output='screen',
    )

    # 3. Robot State Publisher (reads joint_states from joint_state_broadcaster)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 4. ROS <-> Gazebo bridges
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 5. Controller spawners — delayed to give Gazebo time to start the
    #    gz_ros2_control plugin and create the controller_manager node.
    def spawner(name, delay=5.0):
        return TimerAction(
            period=delay,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name, '--controller-manager-timeout', '30'],
                output='screen',
            )],
        )

    # 6. Static world → odom (identity — RViz world frame anchor for Gazebo)
    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        world_tf,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller',       delay=12.0),
    ])
