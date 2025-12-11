# Hector SLAM and Indoor Positioning Robot
ROS Noetic · TurtleBot3 Simulation · Hector SLAM Mapping

This project implements Hector SLAM for indoor mapping using TurtleBot3 (Burger) in Gazebo, along with RViz visualization and keyboard teleoperation.  
The README includes exact tested commands, troubleshooting guides, and recommended improvements for further development.

---

## Table of Contents
- [Project Overview](#project-overview)
- [Quick Start (Exact Tested Commands)](#quick-start-exact-tested-commands)
  - [Terminal 1 — Launch Gazebo Simulation](#terminal-1-—-launch-gazebo-simulation)
  - [Terminal 2 — Launch Hector SLAM](#terminal-2-—-launch-hector-slam)
  - [Terminal 3 — Open RViz](#terminal-3-—-open-rviz)
  - [Terminal 4 — Teleop (Keyboard Control)](#terminal-4-—-teleop-keyboard-control)
- [Key Features](#key-features)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation & Setup](#installation--setup)
- [Running TurtleBot3 Simulation](#running-turtlebot3-simulation)
- [Running Hector SLAM](#running-hector-slam)
- [Running RViz](#running-rviz)
- [Running Teleop](#running-teleop)
- [Recording & Playing ROS Bags](#recording--playing-ros-bags)
- [Troubleshooting](#troubleshooting)
- [Results & Visuals](#results--visuals)
- [Contact](#contact)

---

## Project Overview
This repository contains launch files, configurations, and documentation for running Hector SLAM with a TurtleBot3 Burger in simulation.  
Users can generate indoor maps, observe real-time SLAM in RViz, and operate the robot via teleoperation.

Tested on:
- Ubuntu 20.04  
- ROS Noetic  
- Gazebo (TurtleBot3 World)

---

## Quick Start (Exact Tested Commands)

Open **4 terminals** and run the following. Important: TURTLEBOT3_MODEL must be exported in every terminal using TurtleBot3 (see note at the end).

### Terminal 1 — Launch Gazebo Simulation
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Terminal 2 — Launch Hector SLAM
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch hector_slam_launch turtlebot3_hector_slam.launch
```

### Terminal 3 — Open RViz
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
rosrun rviz rviz
```

### Terminal 4 — Teleop (Keyboard Control)
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Note: If you forget to export TURTLEBOT3_MODEL in any terminal you will see RLException: environment variable 'TURTLEBOT3_MODEL' is not set. Export once per shell or add it to your ~/.bashrc to make it permanent.

---

## Key Features
- Real-time 2D SLAM using Hector Mapping
- TurtleBot3 Burger simulation in Gazebo
- RViz visualization of map, laser scan, and TF tree
- Teleoperation for manual exploration
- Ready for extension to real robot hardware

---

## Repository Structure
Hector_Slam_and_Indoor_Positioning_Robot/
- src/
  - hector_slam_related_packages/
  - positioning_nodes/
  - turtlebot3_simulations/
- README.md
- LICENSE
- (recommended) setup.sh, run_demo.sh, docs/, results/

---

## Prerequisites

Install ROS Noetic + dependencies:
```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full \
                    python3-rosdep \
                    build-essential \
                    python3-catkin-tools
sudo rosdep init
rosdep update
```

Install TurtleBot3 + SLAM packages:
```bash
sudo apt install -y ros-noetic-turtlebot3*
sudo apt install -y ros-noetic-hector-slam
```

---

## Installation & Setup
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# clone this repository here
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

Tip: Add `source ~/catkin_ws/devel/setup.bash` and `export TURTLEBOT3_MODEL=burger` to your ~/.bashrc to avoid repeating them in every terminal:
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

---

## Running TurtleBot3 Simulation
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

---

## Running Hector SLAM
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch hector_slam_launch turtlebot3_hector_slam.launch
```

Hector SLAM typically subscribes to:
- /scan
- /tf
- /odom (optional)

---

## Running RViz
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
rosrun rviz rviz
```

Enable the following displays in RViz:
- Map
- LaserScan
- TF
- Pose

You may also load a saved .rviz config (recommended to add one in the repo under docs/ or results/).

---

## Running Teleop
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Controls:
- W/X = forward / backward
- A/D = left / right rotation
- SPACE = stop

---

## Recording & Playing ROS Bags

Record a mapping session:
```bash
rosbag record -O hector_mapping /scan /tf /odom /imu
```

Replay:
```bash
rosbag play hector_mapping.bag --clock
```

Save a map produced by the SLAM node:
```bash
rosrun map_server map_saver -f results/my_map
```

---

## Troubleshooting

1) Error: TURTLEBOT3_MODEL not set  
- Symptom: RLException: environment variable 'TURTLEBOT3_MODEL' is not set.  
- Fix:
  ```bash
  export TURTLEBOT3_MODEL=burger
  ```
  Make it permanent:
  ```bash
  echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
  ```

2) Warning: xacro in-order processing  
- This is generally a warning and safe to ignore for simulation.

3) RViz showing llvmpipe (software rendering)  
- Indicates no GPU acceleration.  
- Fix: install/update graphics drivers or use a GPU-enabled machine.

4) Hector SLAM diverges or map drifts  
- Ensure the robot's simulated laser scan frame and /tf are published correctly.
- Try adding odometry (/odom) if available, or tune hector parameters for your environment.

---

## Results & Visuals (Recommended)

---

## Contact
Harsha (Harsha-18-H)  
For issues or feature requests, open an issue on GitHub.
