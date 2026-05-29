cmake_minimum_required(VERSION 3.8)
project(robot_3d3s)

find_package(ament_cmake REQUIRED)

# Install all resource directories into the package share folder
install(DIRECTORY
  config
  launch
  meshes
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts so they can be run as nodes
install(PROGRAMS
  scripts/joint_control_gui.py
  scripts/motion_demo.py
  scripts/motion_demo_swerve.py
  scripts/teleop_keyboard.py
  scripts/teleop_keyboard_swerve.py  
  scripts/teleop_keyboard_swerve_gazebo.py  
  scripts/odom_node.py
  scripts/cmd_vel_to_wheels.py
  scripts/cmd_vel_stamper.py  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
