
# COMP30271 Robot Navigation System

This package provides a modular ROS2-based control system for object detection, navigation, wall-following, and logging using camera and odometry inputs. It is compatible with both simulation and JetBot hardware.

## Getting Started

### Setup
Make sure the ROS2 workspace has been built:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_vision_pkg
source install/setup.bash
```

### Launch Full System
To run the complete system:
```bash
ros2 launch my_robot_vision_pkg vision_and_traffic_launch.py
```

---

## Feature Demonstrations

### 1. Mapping (SLAM)
To generate a live map:
```bash
ros2 launch my_robot_vision_pkg rtabmap_slam_launch.py
```
Visualise using:
```bash
rviz2
```

---

### 2. Wall Following
To engage wall-following navigation:
```bash
ros2 run my_robot_vision_pkg wall_follow_node
```

---

### 3. Object Detection
Detect and count visual objects:
```bash
ros2 run my_robot_vision_pkg vision_node
ros2 topic echo /object_counts
```

---

### 4. Goal Detection
To stop the robot on detecting a goal marker:
```bash
ros2 run my_robot_vision_pkg goal_node
```

---

### 5. Traffic Sign Response
Interpret visual commands like STOP/SLOW/SPEED:
```bash
ros2 run my_robot_vision_pkg traffic_node
ros2 run my_robot_vision_pkg control_node
```

---

### 6. Navigate to Goal
To drive the robot to a target coordinate (e.g. 2.0, 2.0):
```bash
ros2 run my_robot_vision_pkg nav_node
```

---

### 7. Landmark Logging
To log object counts with timestamps:
```bash
ros2 run my_robot_vision_pkg landmark_logger
cat /tmp/landmark_log.csv
```

---

### 8. Detection with Instance Counts
Included in vision_node output — use:
```bash
ros2 topic echo /object_counts
```

---

### 9. Visual Odometry
Launch SLAM as in Mapping section and move robot to observe mapping.

---

### 10. Real Robot Compatibility
System connects to JetBot topics:
- `/image_raw` (camera)
- `/odom` (movement)
- `/cmd_vel` (control)

---

## Manual Robot Control (For Testing)
Send direct movement commands:
```bash
# Move forward
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Turn left
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Visual Feedback and Debugging
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /object_counts
```

