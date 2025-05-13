# my_robot_vision_pkg

## Overview
This ROS2 package implements a full robot control pipeline including:
- Real-time object detection using YOLOv5
- Traffic sign interpretation
- Velocity control
- Navigation toward a goal (A* inspired logic)
- Goal detection and stopping

Designed for COMP30271 Cognitive Computing coursework.

---

## Dependencies

Ensure you have the following installed inside your ROS2 workspace or Docker:
```bash
pip install torch torchvision torchaudio
pip install opencv-python numpy pandas
```

---

## Installation

1. Place this folder into your ROS2 workspace `src/` directory:
```bash
cd ~/ros2_ws/src
unzip my_robot_vision_pkg.zip
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_vision_pkg
source install/setup.bash
```

---

## Running the System

Use this launch file to run all components:
```bash
ros2 launch my_robot_vision_pkg vision_and_traffic_launch.py
```

Ensure a camera or rosbag is publishing to `/image_raw`, and odometry is available on `/odom`.

---

## Nodes Included

### `vision_node`
- Subscribes to: `/image_raw`
- Publishes: `/object_counts`
- Detects oranges, trees, vehicles, traffic signs

### `traffic_node`
- Subscribes to: `/object_counts`
- Logs sign-based decisions: STOP, SLOW, SPEED

### `control_node`
- Subscribes to: `/object_counts`
- Publishes: `/cmd_vel`
- Sets robot velocity based on signs

### `nav_node`
- Subscribes to: `/odom`
- Publishes: `/cmd_vel`
- Moves robot toward hardcoded goal (basic A* logic)

### `goal_node`
- Subscribes to: `/object_counts`
- Publishes: `/cmd_vel`
- Stops robot if goal marker is detected

---

## Contact
Created by Frank Motsi (N1007535) for COMP30271 at NTU.
