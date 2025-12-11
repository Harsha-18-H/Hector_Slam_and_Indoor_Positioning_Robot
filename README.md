# Hector SLAM and Indoor Positioning Robot
ROS Noetic · TurtleBot3 Simulation · Hector SLAM Mapping

This project implements Hector SLAM for indoor mapping using TurtleBot3 (Burger) in Gazebo, along with RViz visualization and keyboard teleoperation.  
The README includes exact tested commands, troubleshooting guides, and recommended improvements for further development.

---

## Table of Contents
- [Project Overview](#project-overview)
- [Quick Start (Exact Tested Commands)](#quick-start-exact-tested-commands)
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
- [Roadmap](#roadmap)
- [License](#license)
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

Open **4 terminals** and run the following:

### Terminal 1 — Launch Gazebo Simulation
```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch


