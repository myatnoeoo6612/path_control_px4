🚀 PX4 ROS 2 Drone Control

This repository provides a full pipeline for controlling a PX4 SITL drone with ROS 2, including Gazebo simulation, Micro XRCE-DDS communication, QGroundControl monitoring, and a ROS 2 trajectory flight controller.

The system supports:

PX4 SITL (Gazebo Harmonic)

ROS 2 Humble

MicroXRCE-DDS Agent

QGroundControl

Custom ROS 2 flight controller (trajectory_fly)

📦 System Overview
PX4 SITL  →  MicroXRCE-DDS Agent  →  ROS 2 Nodes (Bringup)  →  Control Node
         ↘———— QGroundControl (monitoring & MAVLink)

🛠 Installation

Make sure PX4, ROS 2 Humble, MicroXRCEAgent, and QGroundControl are installed properly.

Clone this repository into your ROS 2 workspace:

cd ~/ros2_ws/src
git clone <your-repo-url>
cd ..
colcon build

▶️ How to Run the Drone System

Follow these steps in order:

Step 1 — Start PX4 SITL (Gazebo X500 model)
make px4_sitl gz_x500


This launches PX4 SITL using Gazebo Harmonic with the standard X500 quadrotor.

Step 2 — Start Micro XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888


PX4 <-> ROS 2 communication uses XRCE-DDS through port 8888.

Step 3 — Run QGroundControl

(Optional but recommended)

./QGroundControl


Used for:

Monitoring PX4 status

Viewing MAVLink messages

Checking RC, EKF, battery, etc.

Step 4 — Source your ROS 2 workspace
source install/setup.bash


Do this inside your workspace (e.g., ~/ros2_ws).

Step 5 — Launch ROS 2 Drone Bringup
ros2 launch bringup_drone bringup_launch.launch.py 


Starts:

PX4 bridge topics

TF broadcaster

State/odometry nodes

Sensor transforms

Step 6 — Run the Trajectory Flight Controller
ros2 run control trajectory_fly


This node performs:

Takeoff

Fly to (0,0,-5)

3-second hover

Square pattern flight

Heading rotation at each corner

Return to home

Autonomous landing

📂 Repository Structure
.
├── bringup_drone/
│   ├── launch/
│   └── config/
│
├── control/
│   ├── src/trajectory_fly.cpp
│   ├── include/
│   └── CMakeLists.txt
│
├── README.md
└── package.xml