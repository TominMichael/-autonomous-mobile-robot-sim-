# ROS 2 Autonomous Wheelchair Simulation

![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![ROS 2: Humble](https://img.shields.io/badge/ROS%202-Humble-blue)
![Gazebo: Fortress](https://img.shields.io/badge/Gazebo-Fortress-orange)
![Status: Development](https://img.shields.io/badge/Status-Development-green)

This repository contains the **simulation package** for the Autonomous Wheelchair Project. It provides a comprehensive Gazebo simulation environment with a differential drive wheelchair model, outfitted with Lidar and IMU sensors, enabling development and testing of navigation algorithms (SLAM, AMCL, Nav2) without physical hardware.

## 🚀 Features

- **High-Fidelity URDF Model**: Realistic wheelchair chassis with differential drive kinematics.
- **Sensor Simulation**:
  - **Lidar**: 2D Laser Scan for mapping and obstacle avoidance.
  - **IMU**: Accelerometer and Gyroscope for sensor fusion (EKF).
  - **Encoders**: Simulated wheel odometry.
- **Gazebo Integration**: Custom worlds (Maze, Office) for testing navigation scenarios.
- **Navigation Stack**: Pre-configured launch files for `slam_toolbox` (Mapping) and `nav2` (Autonomous Navigation).
- **RViz Visualization**: Real-time visualization of robot state, map, and sensor data.

## 🛠️ Prerequisites

- **ROS 2 Humble Hawksbill** (Desktop Full typically recommended)
- **Gazebo Fortress** (Ignition Gazebo)
- **Navigation2**
- **SLAM Toolbox**

### Install Dependencies
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
sudo apt install ros-humble-ros-gz ros-humble-xacro
```

## 🔧 Installation

1. **Create Workspace** (if not exists):
   ```bash
   mkdir -p ~/wheelchair_ws/src
   cd ~/wheelchair_ws/src
   ```

2. **Clone Repository**:
   ```bash
   git clone https://github.com/TominMichael/ros2-wheelchair-simulation.git wheelchair
   ```

3. **Build Package**:
   ```bash
   cd ~/wheelchair_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## 📡 Usage

### 1. Launch Simulation (Gazebo + RViz)
Starts the robot in a maze world with sensor bridges active.
```bash
ros2 launch wheelchair launch.py
```

### 2. Run SLAM (Mapping)
Launches SLAM Toolbox to generate a map of the environment.
```bash
ros2 launch wheelchair slam.launch.py
```
*Drive the robot using teleop to build the map.*

### 3. Autonomous Navigation (Nav2)
Launches the Navigation2 stack with AMCL for localization.
```bash
ros2 launch wheelchair navigation.launch.py
```
*Use "2D Pose Estimate" to localize and "Nav2 Goal" to send commands in RViz.*

## 📂 Project Structure

```
├── config/         # Parameters for Nav2, SLAM, and EKF
├── launch/         # Python launch files
├── maps/           # Saved maps (.pgm/.yaml)
├── meshes/         # 3D assets (STL/DAE)
├── models/         # Gazebo SDF models
├── urdf/           # Robot description (Xacro)
├── worlds/         # Gazebo simulation environments
└── package.xml     # Dependencies
```

## 🤝 Contributing

Contributions are welcome! Please open an issue or submit a pull request.

1. Fork the repo.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.

## ✍️ Authors

- **Tomin Michael** - *Initial Work*
