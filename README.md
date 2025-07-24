# jellydrone2025

# ROS2 Setup for Jellydrone IMU Visualization

This guide walks you through uploading your Arduino code, launching the ROS 2 nodes, running the IMU filter, and visualizing IMU movement using RViz.

---

## Prerequisites

- Ubuntu with ROS 2 installed
- Arduino IDE
- A properly configured `jellydrone_ws` ROS 2 workspace
- Arduino board with IMU code uploaded

---

## Step 1: Upload Arduino Code

1. Open the **Arduino IDE** on Ubuntu.
2. Upload the IMU code to your Arduino board.

---

## Step 2: Launch ROS 2 Environment

### Terminal 1: Launch Main ROS2 Node

```bash
cd ~/jellydrone_ws
colcon build
source install/setup.bash
ros2 launch jellydrone jellydrone.launch.py

```

### Terminal 2: Run IMU Filter Node

```bash
cd ~/jellydrone_ws
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=false \
  -r imu/data_raw:=/imu/data_raw \
  -r imu/data:=/imu/data_filtered

```

### Terminal 3: Launch RViz

```bash
cd ~/jellydrone_ws
rviz2

```
