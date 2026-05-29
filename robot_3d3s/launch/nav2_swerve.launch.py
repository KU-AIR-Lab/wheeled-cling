"""
nav2_swerve.launch.py  -  Navigation2 launch for the 3D3S robot using swerve_controller.

Launches the full navigation stack in either RViz-only or Gazebo mode.

RViz mode (default - no physics, pure visualization):
  ros2 launch robot_3d3s nav2_swerve.launch.py

Gazebo mode (real physics):
  ros2 launch robot_3d3s nav2_swerve.launch.py gazebo:=true

What this launches
------------------
Both modes:
  - robot_state_publisher    (URDF -> TF)
  - odom_node                (/joint_states -> /odom + TF odom->base_footprint)
  - nav2 nodes               (planner, controller, bt_navigator, ...)
  - RViz2                    (with nav2 panel for setting goals)
  - static TF: map -> odom   (identity - nav2 needs the map frame)

RViz mode only:
  - cmd_vel_to_wheels        (/cmd_vel -> /joint_states visualization)

Gazebo only:
  - Gazebo Sim               (physics)
  - ros2_control controllers (joint_state_broadcaster + swerve_controller)
  - ros_gz_bridge            (/clock)
  - cmd_vel_stamper.py       (/cmd_vel Twist -> /swerve_controller/cmd_vel TwistStamped)

cmd_vel pipeline in Gazebo mode
-------------------------------
  controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
  -> cmd_vel_stamper.py -> /swerve_controller/cmd_vel -> swerve_controller -> robot

This keeps swerve_controller.yaml compatible with:
  use_stamped_vel: true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    world_path = os.path.join(pkg, 'worlds', 'my_world.sdf')
    nav2_rviz_config = os.path.join(pkg, 'config', 'nav2.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc_gz = robot_desc.replace('CONTROLLERS_YAML_PATH', controllers_yaml)

    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='false',
        description='true = use Gazebo physics, false = RViz-only',
    )
    gazebo = LaunchConfiguration('gazebo')

    # Shared nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen',
    )

    odom_node = Node(
        package='robot_3d3s',
        executable='odom_node.py',
        name='odom_node',
        output='screen',
    )

    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    params = [nav2_params_file, {'use_sim_time': True}]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    route_server = Node(
        package='nav2_route',
        executable='route_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=params,
        remappings=tf_remaps + [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    cmd_vel_stamper = Node(
        package='robot_3d3s',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/swerve_controller/cmd_vel'),
        ],
        condition=IfCondition(gazebo),
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'route_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'docking_server',
            ]},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # RViz-only mode: keep the old visualization bridge.
    cmd_vel_rviz = Node(
        package='robot_3d3s',
        executable='cmd_vel_to_wheels.py',
        name='cmd_vel_to_wheels',
        parameters=[{'gazebo': False}],
        output='screen',
        condition=UnlessCondition(gazebo),
    )

    # Gazebo mode
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
        condition=IfCondition(gazebo),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot_3d3s',
            '-string', robot_desc_gz,
            '-x', '0', '-y', '0', '-z', '0.2',
        ],
        output='screen',
        condition=IfCondition(gazebo),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(gazebo),
    )

    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '30'],
            output='screen',
            condition=IfCondition(gazebo),
        )])

    return LaunchDescription([
        gazebo_arg,

        # Shared
        robot_state_publisher,
        world_to_map,
        map_to_odom,
        odom_node,

        # Nav2 nodes
        controller_server,
        smoother_server,
        planner_server,
        route_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        cmd_vel_stamper,
        docking_server,
        lifecycle_manager,

        rviz,

        # RViz mode
        cmd_vel_rviz,

        # Gazebo mode
        gz_sim,
        spawn_robot,
        clock_bridge,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
    ])
