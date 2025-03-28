# UGV ROS Project 🤖

This repository contains the development files for an Unmanned Ground Vehicle (UGV) built from scratch using ROS on a Jetson Nano. The robot supports remote manual and autonomous navigation, sensor streaming, and motor control using a custom Arduino driver.

---

## 🔧 Features

- Manual and autonomous control via ROS
- Real-time streaming of camera, IMU, and 2D LiDAR data
- Navigation using `move_base` through RViz click-to-goal
- Arduino motor control via I2C and ROS Serial
- Custom Gazebo simulation environment
- Self-built robot platform and control stack
- Functional integration with `rqt_robot_steering`

---

## 📁 Project Structure
1. Clone the Repository into Your Catkin Workspace
```bash
UGV-ROS-Project/
├── jetson_nano_bot/         # Main robot navigation and control package
├── ros-imu-bno055/          # IMU interface (BNO055) package
├── rplidar_ros/             # 2D LIDAR (RPLIDAR) interface package
├── README.md                # You're here!
```



🚀 Setup & Launch Instructions
1. Clone the Repository into Your Catkin Workspace
```bash
cd ~/catkin_ws/src
git clone https://github.com/ali-qdmz/UGV-ROS-Project.git
cd ..
catkin_make
source devel/setup.bash
```
2. Launch the UGV Robot Stack
```bash
roslaunch navstack_pub jetson_nano_bot.launch
```
3. Grant I2C Permission for Arduino Motor Control
```bash
sudo chmod 666 /dev/i2c-3
```
4. Launch the Intel RealSense Camera
```bash
roslaunch realsense2_camera rs_camera.launch
```
🎮 Manual Control (Optional)
You can use rqt_robot_steering to manually control the robot:
```bash
rosrun rqt_robot_steering rqt_robot_steering
```
✅ The Arduino motor controller is programmed to interface with ROS via I2C and receive velocity commands directly.


📹 Demo Videos

Real Lab Test:

<div align="center">
<video src="https://github.com/user-attachments/assets/a4314669-bbcf-40fc-9f6c-99f1033c38df" width="352" height="720"></video>
</div>


<div align="center">
<video src="https://github.com/user-attachments/assets/c4012d78-f0d0-400a-b571-0f7993356586" width="352" height="720"></video>
</div>

<div align="center">
<video src="https://github.com/user-attachments/assets/afbd2eb6-08d8-4cf9-ad78-cef5e8ea95d6" width="352" height="720"></video>
</div>

Simulation 
<div align="center">
<video src="https://github.com/user-attachments/assets/6b1c939a-f970-413c-af52-2a7a30d0e109" width="352" height="720"></video>
</div>

## 👨‍💻 Author
 - Ali Ghadimzadeh Alamdari
 - Ph.D. Candidate, Drexel University
 - Electrical & Computer Science Team Lead @ Lubabotics (2023)

