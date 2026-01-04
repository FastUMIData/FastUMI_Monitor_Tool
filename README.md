# ROS Topic Monitoring Script User Guide

## 📋 Project Overview

This is an interactive script for monitoring and visualizing data from **XV SDK** devices. Key features include:
- Monitor the data rate (Hz) of various sensors
- View sensor message contents in real time
- Launch RViz for visualization
- Support parallel execution across multiple terminals

## 🚀 Quick Start

### Requirements
- Ubuntu 20.04
- ROS Noetic

### Prerequisites
1. **Environment (strongly recommended: create a virtual environment with the same name to avoid script errors)**
   ```bash
   conda create -n fastumi python=3.8.5
   conda activate fastumi
   pip install -r requirements.txt
   ```
---

### Installation & Running
0. **Start the camera**
   ```bash
   roslaunch xv_sdk xv_sdk.launch
   ```

1. **Make the script executable**
   ```bash
   chmod +x fastumi_monitor_menu.sh
   ```

2. **Run the script**
   ```bash
   bash fastumi_monitor_menu.sh
   ```

3. **Basic usage**
   - A menu will appear after launching
   - Enter the corresponding number to execute the selected function
   - Enter `0` to exit

## 📊 Feature Details

### Sensor Frequency Monitoring (1–10)

| ID | Function | What It Monitors |
|------|------|----------|
| 1 | IMU frequency monitor | `slam/pose` topic rate |
| 2 | Read IMU data | `slam/pose` topic content |
| 3 | SLAM frequency monitor | `slam/visual_pose` topic rate |
| 4 | Read SLAM data | `slam/visual_pose` topic content |
| 5 | RGB camera frequency | `color_camera/camera_info` rate |
| 6 | Front-left fisheye camera frequency | `fisheye_cameras/left/camera_info` rate |
| 7 | Upper-left fisheye camera frequency | `fisheye_cameras/left2/camera_info` rate |
| 8 | Front-right fisheye camera frequency | `fisheye_cameras/right/camera_info` rate |
| 9 | Upper-right fisheye camera frequency | `fisheye_cameras/right2/camera_info` rate |
| 10 | TOF camera frequency | `tof_camera/camera_info` rate |
| 21 | Read clamp data | `/clamp/Data` topic content |

### RViz Visualization (11–20)

| ID | Function | Config File |
|------|------|----------|
| 11 | Four-fisheye view | `four_fisheyes.rviz` |
| 12 | Front-left fisheye | `fisheye_left.rviz` |
| 13 | Upper-left fisheye | `fisheye_left2.rviz` |
| 14 | Front-right fisheye | `fisheye_right.rviz` |
| 15 | Upper-right fisheye | `fisheye_right2.rviz` |
| 16 | RGBD camera | `rgbd_camera.rviz` |
| 17 | RGB camera | `rgb_camera.rviz` |
| 18 | TOF sensor | `tof.rviz` |
| 19 | SLAM visualization | `slam_visualization.rviz` |
| 20 | Full visualization | `general.rviz` |

## 💡 Advanced Tips

### Batch Execution

The script supports multiple input formats:

```bash
# Single ID
1

# Comma-separated
1,3,5

# Ranges (both ascending and descending are supported)
1-5
5-1

# Mixed
1,3-5,10
```

### Parallel Monitoring

- You can launch multiple monitoring windows at the same time
- Each function runs in a **new terminal window**
- Supports monitoring multiple sensors simultaneously

### Automatic Environment Setup

The script will automatically source the following ROS environments:
- `/opt/ros/noetic/setup.bash`
- `/opt/ros/melodic/setup.bash`
- `~/catkin_ws/devel/setup.bash`
- `~/ros_ws/devel/setup.bash`

## 📝 Examples

### Example 1: Monitor IMU data
```bash
# Input: 1,2
# Result: monitor IMU topic rate and view IMU messages simultaneously
```

### Example 2: Launch multiple camera views
```bash
# Input: 11-15
# Result: launch all fisheye camera RViz views
```

### Example 3: Full monitoring
```bash
# Input: 1-10
# Result: monitor the rates of all sensor topics
```

### Example 4: Mixed usage
```bash
# Input: 1,3-5,11,20
# Result: monitor IMU rate, SLAM rate, RGB camera rate; launch four-fisheye view and full visualization
```

## 📝 Reference Metrics
1. **slam/pose**: `rostopic hz SNXXXX/slam/pose` — standard rate is **500 Hz**
2. **color_camera/image_color**: `rostopic hz SNXXXX/color_camera/image_color` — standard rate is **60 Hz**
3. **tof_camera/image**: `rostopic hz SNXXXX/tof_camera/image` — standard rate is **30 Hz**
4. **clamp/Data**: `rostopic echo SNXXXX/clamp/Data` — normal value range is **0–88**, dynamically changing with the Fast UMI gripper opening/closing degree
5. **rviz**: use RViz to verify that camera images at each position are clear and show no obvious stuttering

## ⚠️ Notes

1. **Device connection**: Make sure the XV SDK device is properly connected
2. **ROS environment**: Ensure the ROS environment is correctly configured
3. **Permissions**: Make sure the script has execution permission
4. **Terminal windows**: You can stop any newly opened terminal with Ctrl-C, or simply close the window
5. **Config files**: Make sure RViz config files exist at the specified path

## 🛠️ Troubleshooting

### Common Issues

#### 1. Terminal program not found
```bash
# Install gnome-terminal
sudo apt install gnome-terminal

# Or install another terminal emulator
sudo apt install konsole
sudo apt install xfce4-terminal
sudo apt install tilix
```

#### 2. ROS environment not sourced
```bash
# Manually source environments
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Check ROS environment
echo $ROS_PACKAGE_PATH
```

#### 3. Device ID mismatch
```bash
# Check device connection status
lsusb | grep -i xv

# Inspect actual topic names
rostopic list | grep xv_sdk

# Modify TOPIC_PREFIX in the script
```

#### 4. RViz config file missing
```bash
# Check RViz config path
ls -la /home/onestar/catkin_ws/src/xv_sdk/rviz/

# If the path doesn't exist, modify RVIZ_PATH in the script
```

#### 5. Permission issues
```bash
# Add execution permission to the script
chmod +x run_rostopic_menu.sh

# Check file permissions
ls -la run_rostopic_menu.sh
```

## 🎯 Best Practices

1. **First-time use**: start with a single function to validate setup
2. **Performance monitoring**: use rate monitoring to verify stable data streams
3. **Visual debugging**: combine RViz views with topic monitoring
4. **Batch operations**: use range inputs to improve efficiency
5. **Device debugging workflow**: check frequency first, then inspect message contents
6. **Multi-window management**: use multiple terminals strategically for parallel monitoring

**Tip**: In any newly opened terminal window, press **Ctrl-C** to stop, or simply close the window.
