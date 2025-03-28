# UGV ROS Project 🚗🤖

This project showcases the development of an autonomous Unmanned Ground Vehicle (UGV) using ROS. Built from scratch, the robot integrates sensors (camera, IMU, and 2D LiDAR), a custom Arduino motor controller, and uses the `move_base` package for navigation.

## 🔧 Features

- Manual and autonomous control via ROS
- Real-time video streaming and sensor data
- RViz navigation via `move_base`
- Arduino motor control via I2C
- Custom Gazebo simulation environment
- ROS-compatible URDF and config files


## 📸 Media

<table>
  <tr>
    <td><img src="docs/images/ugv_photo.jpg" width="300"/></td>
    <td><img src="docs/images/simulation_rviz.png" width="300"/></td>
  </tr>
</table>

## 🎥 Demonstration Video
A demonstration of the robotic crack measurement system can be seen below:

<div align="center">
<video src="https://github.com/user-attachments/assets/706ef7f4-40e0-4029-8845-91a72196641b05" width="352" height="720"></video>
</div>

## 📁 Directory Overview

- `arduino/`: Code for Arduino-based motor controller using I2C and ROS Serial
- `ros_ws/`: Catkin workspace with UGV ROS packages
- `simulation/`: Custom Gazebo worlds and launch files
- `docs/`: Documentation, images, and demo videos
- `hardware/`: Specs and wiring diagrams for UGV components

## 🛠️ Setup

```bash
# Clone repo
git clone https://github.com/yourusername/ugv_ros_project.git
cd ugv_ros_project/ros_ws
catkin_make
source devel/setup.bash
roslaunch ugv_navigation bringup.launch
