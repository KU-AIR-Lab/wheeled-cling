# Independently Steered 3-Wheeled Cling Robot

A research repository for the wheeled Cling robot — a 3-wheel-driving, 3-wheel-steering (3D3S) omnidirectional mobile robot.

## Repository Structure

```
wheeled-cling/
├── kinematics/              MATLAB inverse kinematics script
└── robot_3d3s/              ROS 2 package (URDF, simulation, control, navigation)
    ├── config/              RViz configs, controller params, nav2 params
    ├── launch/              Launch files for every simulation mode
    ├── meshes/              STL mesh files for all robot links
    ├── scripts/             Python ROS 2 nodes
    ├── urdf/                Robot URDF description
    └── worlds/              Gazebo world file
```

---

## Requirements

| Dependency | Version |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8.x) |
| ros2_control | 4.x |
| gz_ros2_control | Jazzy |
| Navigation2 | 1.3.x |

Install ROS 2 dependencies:

```bash
sudo apt install \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-mppi-controller \
  python3-numpy \
  python3-pyqt5
```

---

## Build
After cloning and before building, make sure that all the Python files have executable permission. If not, perform chmod +x to them.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/KU-AIR-Lab/wheeled-cling.git
cd ~/ros2_ws
colcon build --packages-select robot_3d3s
source install/setup.bash
```

---

## Launch Modes

### 1. RViz display (URDF viewer, no physics)

```bash
ros2 launch robot_3d3s display.launch.py
```
Use the joint sliders to move each steering/wheel joint manually.

---

### 2. Motion demo — RViz only

Cycles through Rotate CW/CCW → Forward → Backward → Slide Left/Right automatically.

```bash
ros2 launch robot_3d3s motion_test.launch.py
```

---

### 3. Gazebo simulation (physics)

```bash
ros2 launch robot_3d3s gazebo.launch.py
```

Wait ~15 s for controllers to spawn, then the robot is ready.

---

### 4. Keyboard teleoperation

**RViz mode:**
```bash
ros2 launch robot_3d3s teleop.launch.py
```

**Gazebo mode** (run gazebo.launch.py first, then in a second terminal):
```bash
ros2 run robot_3d3s teleop_keyboard.py --ros-args -p gazebo:=true
```

| Key | Motion |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Rotate CCW |
| `d` | Rotate CW |
| `q` | Slide left |
| `e` | Slide right |
| `Space` | Stop |

---

### 5. Autonomous navigation (Nav2)

**RViz only (no physics):**
```bash
ros2 launch robot_3d3s nav2.launch.py
```

**RViz + Gazebo:**
Here Gazebo will be a co-simulation along with Rviz.
```bash
ros2 launch robot_3d3s nav2.launch.py gazebo:=true
```

Once RViz opens, click **"2D Goal Pose"** and click anywhere on the grid.  
Nav2 plans a path and the robot drives autonomously using the MPPI holonomic controller.

---

## Scripts

| Script | Description |
|--------|-------------|
| `motion_demo.py` | Automatic kinematic motion sequence demo |
| `joint_control_gui.py` | PyQt5 GUI — live sliders for each joint |
| `teleop_keyboard.py` | Keyboard teleoperation with smart-flip IK |
| `cmd_vel_to_wheels.py` | Converts `/cmd_vel` Twist → steering + wheel commands |
| `odom_node.py` | Dead-reckoning odometry from `/joint_states` → `/odom` + TF |

---

## Robot Description

**Configuration:** 3-wheel-drive, 3-wheel-steer (3D3S) omnidirectional  
**Wheel layout:** 3 legs at 0°, 120°, 240°  
**Leg radius:** 389 mm | **Wheel radius:** 50 mm

### Inverse Kinematics

Given body velocity `[vx, vy, ω]`, the steering angle `φᵢ` and wheel speed `ωᵢ` for wheel `i` are:

```
vwx_i = vx − sin(αᵢ) · ω · R_contact
vwy_i = vy + cos(αᵢ) · ω · R_contact

φᵢ   = atan2(vwy_i, vwx_i) − π/2   (clipped to [−π/2, π/2])
ωᵢ   = ‖[vwx_i, vwy_i]‖ / R_wheel
```

where `αᵢ ∈ {0°, 120°, 240°}` are the leg angles.  
See `kinematics/InverseKinematics.m` for the MATLAB reference implementation.

**Smart-flip optimisation:** for each wheel the IK produces two equivalent solutions `(φ, ω)` and `(φ±π, −ω)`. The solution closest to the current steering angle is chosen, so Forward→Backward simply reverses wheel spin without moving the steering shafts.

---

## Author

Yersaiyn Bushanovy — Nazarbayev University  
KU AIR Lab
