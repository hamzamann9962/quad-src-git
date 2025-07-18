# robot dog v1 Simulation and Inverse Kinematics

This repository contains ROS 2 packages for simulating and controlling a quadruped robot, including inverse kinematics, trajectory generation, and hardware integration.

## 🧭 Repository Structure

| Branch           | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| `main`           | Simulation of a **single leg** of the quadruped robot.                      |
| `full_quad`      | Complete simulation of the **full quadruped robot**.                        |
| `quad_hardware`  | **Hardware integration** of the quadruped robot using `ros2_control` on devices like Jetson Nano or Raspberry Pi. |

## 📦 Packages

1. **quad_description** – URDF, Gazebo, and RViz setup for the robot.  
2. **ik** – C++ implementation of analytical inverse kinematics for a leg.

## 🚀 Installation

```bash

git clone https://github.com/hamzamann9962/quad-src-git.git
cd ~/quad-src-git
colcon build --packages-select quad_description ik
source install/setup.bash
```
for main branch :

```bash
# Launch Gazebo and ros2_control for single leg
ros2 launch quadruped_description gazebo.launch.py

# Run custom IK service
ros2 run ik ik_service

# Run trajectory planning and execution
ros2 run quadruped_description traj_bezier

# Visualize in RViz with joint GUI
ros2 launch quadruped_description display.launch.py
```
For full_quad branch :
```bash
git checkout full_quad
colcon build
source install/setup.bash
```
```bash
ros2 launch quadruped_description gazebo.launch.py
ros2 run ik ik
ros2 run quadruped_description traj_bezier
ros2 launch quadruped_description display.launch.py
```
🤖 Hardware Integration (quad_hardware branch)
This branch deploys the robot on actual hardware (e.g., Jetson Nano, Raspberry Pi).
(simulation time disabled in rpi but if u wanna try on your laptop u can run this 3 comands
but on rpi just use the first comand to run the hardware lunch , i make it to make the rpi work smother withouth simulation)

```bash
# Launch hardware interface 
ros2 launch quadruped_description hardware.launch.py
ros2 launch quadruped_description gazebo.launch.py
ros2 launch quadruped_description display.launch.py 

# Run trajectory planning and execution
ros2 run quadruped_description traj_bezier

# Launch LiDAR data publisher
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

# Launch joystick control
ros2 launch quadruped_description joystick.launch.py
```



