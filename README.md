# ROS2 Topic Monitor Script Guide

[English](README.md) | [中文](README_CN.md)

## 📋 Project Overview

This project provides an interactive script to monitor and visualize XV SDK device data (ROS2 Galactic). Key features:
- Monitor sensor message rates (Hz)
- View sensor message contents in real time
- Launch RViz2 visualization
- Support running multiple terminals in parallel

## 🚀 Quick Start

### Requirements
- Ubuntu 20.04
- ROS2 Galactic

### Prerequisites

1. **Create a Python virtual environment (conda recommended)**
   ```bash
   conda create -n fastumi python=3.8.5
   conda activate fastumi
   pip install -r requirements.txt
   ```

2. **Install ROS2 dependencies**
   ```bash
   sudo apt install ros-galactic-rviz2
   ```

---

### Install & Run

#### 0. Start the XV SDK ROS2 node
```bash
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py
```

#### 1. Ensure scripts are executable
```bash
chmod +x fastumi_monitor_menu.sh
chmod +x single_fastumi_monitor_menu.sh
chmod +x run_ros2topic_menu.sh
chmod +x rviz/scripts/*.sh
```

#### 2. Run the monitoring script
```bash
# Auto-detect all devices and open monitoring windows
bash fastumi_monitor_menu.sh

# Or specify a single device (serial number must include the "SN" prefix)
bash single_fastumi_monitor_menu.sh SN250801DR48FP25002587
```

#### 3. Basic usage
- After launching, a menu will appear
- Enter the corresponding number to run a function
- Enter `0` to exit

## 📊 Features

### Sensor Rate Monitoring (1-10)

| No. | Function | Topic monitored |
|---:|----------|----------------|
| 1 | IMU rate | `imu` topic rate |
| 2 | IMU echo | `imu` topic content |
| 3 | SLAM rate | `pose` topic rate |
| 4 | SLAM echo | `pose` topic content |
| 5 | RGB camera rate | `rgb/image` rate |
| 6 | Front-left fisheye rate | `fisheye_cameras_left/camera_info` rate |
| 7 | Upper-left fisheye rate | `fisheye_cameras_left2/camera_info` rate |
| 8 | Front-right fisheye rate | `fisheye_cameras_right/camera_info` rate |
| 9 | Upper-right fisheye rate | `fisheye_cameras_right2/camera_info` rate |
| 10 | TOF camera rate | `tof/depth/image_rect_raw` rate |
| 21 | Clamp echo | `clamp` topic content |

### RViz2 Visualization (11-20)

| No. | View | Config file |
|---:|------|-------------|
| 11 | Four fisheyes | `four_fisheyes.rviz` |
| 12 | Front-left fisheye | `fisheye_left.rviz` |
| 13 | Upper-left fisheye | `fisheye_left2.rviz` |
| 14 | Front-right fisheye | `fisheye_right.rviz` |
| 15 | Upper-right fisheye | `fisheye_right2.rviz` |
| 16 | RGBD camera | `rgbd_camera.rviz` |
| 17 | RGB camera | `rgb_camera.rviz` |
| 18 | TOF sensor | `tof.rviz` |
| 19 | SLAM visualization | `slam_pose_markers.rviz` |
| 20 | Overall visualization | `general.rviz` |

## 💡 Advanced Tips

### Batch execution

The menu supports multiple input formats:

```bash
# Single number
1

# Comma-separated
1,3,5

# Ranges (ascending and descending)
1-5
5-1

# Mixed
1,3-5,10
```

### Parallel monitoring

- You can launch multiple monitoring windows at the same time
- Each function runs in a new terminal window
- Supports monitoring multiple sensors concurrently

### Automatic environment setup

The script will try to source the following ROS2 environments automatically:
- `/opt/ros/galactic/setup.bash`
- `~/ros2_ws/install/setup.bash`
- `~/colcon_ws/install/setup.bash`

**Note**: In ROS2, the pose topic uses standard `geometry_msgs/PoseStamped` and does not include the ROS1 `confidence` field.

## 📝 Reference Metrics

1. **pose**: `ros2 topic hz /xv_sdk/<SN>/pose` nominal rate is 500 Hz
2. **rgb/image**: `ros2 topic hz /xv_sdk/<SN>/rgb/image` nominal rate is 60 Hz
3. **tof/depth/image_rect_raw**: `ros2 topic hz /xv_sdk/<SN>/tof/depth/image_rect_raw` nominal rate is 30 Hz
4. **clamp**: `ros2 topic echo /xv_sdk/<SN>/clamp` nominal value range is 0–88

Example: `ros2 topic hz /xv_sdk/SN250801DR48FP25002587/pose`

5. **rviz2**: check whether each camera view is clear and smooth, with no obvious stutter

## ⚠️ Notes

1. **Device connection**: Ensure the XV SDK device is properly connected
2. **ROS2 environment**: Ensure ROS2 Galactic is correctly set up
3. **Permissions**: Ensure scripts have execute permissions
4. **Terminal windows**: In newly opened terminals, press Ctrl-C to stop, or simply close the window
5. **Config files**: RViz2 config files are generated automatically based on the device serial number

## 🛠️ Troubleshooting

### Common issues

#### 1. Terminal emulator not found
```bash
# Install gnome-terminal
sudo apt install gnome-terminal

# Or install another terminal
sudo apt install konsole
sudo apt install xfce4-terminal
sudo apt install tilix
```

#### 2. ROS2 environment not loaded
```bash
# Manually source environments
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash

# Check ROS2 environment
echo $ROS_DISTRO
ros2 topic list
```

#### 3. Device ID mismatch
```bash
# Check device connection status
lsusb | grep -i xv

# List actual topic names
ros2 topic list | grep xv_sdk
```

#### 4. RViz2 config file not found
```bash
# Manually generate config files (serial number must include the "SN" prefix)
bash rviz/scripts/generate_configs.sh SN250801DR48FP25002587

# Check the generated config path
ls -la rviz/generated/<device_serial>/
```

#### 5. Permission issues
```bash
# Add execute permissions
chmod +x *.sh
chmod +x rviz/scripts/*.sh
```

## 🎯 Best Practices

1. **First time**: Start by testing a single feature
2. **Performance monitoring**: Use rate monitoring to check the data stream
3. **Visual debugging**: Combine RViz2 views for visual inspection
4. **Batch operations**: Use range inputs to improve efficiency
5. **Device debugging**: Check rates first, then inspect message contents
6. **Multi-window management**: Use multiple terminal windows for parallel monitoring

**Tip**: In newly opened terminals, press Ctrl-C to stop, or simply close the window.
