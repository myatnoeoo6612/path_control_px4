🚀 PX4 ROS 2 Drone Control


## Setup Instructions

### 1. Micro XRCE Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 2. Drone Bring-Up

```bash
ros2 launch bringup_drone bringup_launch.launch.py
```

#### 3. Fly
```bash
ros2 run control trajectroy_fly 
```


