# ROS2 Galactic XV SDK 监控工具

## 📋 项目简介

这是一个用于监控和可视化XV SDK设备数据的交互式工具（ROS2 Galactic版本），主要功能包括：
- 监控各种传感器的数据频率（Hz）
- 实时查看传感器数据内容
- 启动RViz2可视化界面
- 支持多终端并行运行
- 自动检测多设备并分别管理

## 🚀 快速开始

### 环境要求
- Ubuntu 20.04 / 22.04
- ROS2 Galactic / Humble
- Python 3.8+

### 前置条件

1. **创建Python虚拟环境（推荐使用conda）**
   ```bash
   conda create -n fastumi python=3.8.5
   conda activate fastumi
   pip install -r requirements.txt
   ```

2. **安装ROS2依赖**
   ```bash
   sudo apt install ros-galactic-rviz2
   # 或者 for Humble:
   # sudo apt install ros-humble-rviz2
   ```

---

### 安装与运行

#### 0. 启动XV SDK ROS2节点
```bash
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py
```

#### 1. 确保脚本有执行权限
```bash
chmod +x fastumi_monitor_menu.sh
chmod +x single_fastumi_monitor_menu.sh
chmod +x run_ros2topic_menu.sh
chmod +x rviz/scripts/*.sh
```

#### 2. 运行监控脚本
```bash
# 自动检测所有设备并打开监控窗口
bash fastumi_monitor_menu.sh

# 或者指定单个设备（序列号带SN前缀）
bash single_fastumi_monitor_menu.sh SN250801DR48FP25002587
```

#### 3. 基本使用
- 运行后会出现菜单界面
- 输入对应编号即可执行相应功能
- 输入 `0` 退出脚本

## 📊 功能详解

### 传感器频率监控 (1-10)

| 编号 | 功能 | 监控内容 |
|------|------|----------|
| 1 | imu频率监控 | `imu` 话题频率 |
| 2 | imu数据读取 | `imu` 话题内容 |
| 3 | SLAM频率监控 | `pose` 话题频率 |
| 4 | SLAM数据读取 | `pose` 话题内容 |
| 5 | RGB相机频率 | `rgb/image` 频率 |
| 6 | 左前鱼眼相机频率 | `fisheye_cameras_left/camera_info` 频率 |
| 7 | 左上鱼眼相机频率 | `fisheye_cameras_left2/camera_info` 频率 |
| 8 | 右前鱼眼相机频率 | `fisheye_cameras_right/camera_info` 频率 |
| 9 | 右上鱼眼相机频率 | `fisheye_cameras_right2/camera_info` 频率 |
| 10 | TOF相机频率 | `tof/depth/image_rect_raw` 频率 |
| 21 | 夹具数据读取 | `clamp` 话题内容 |

### RViz2可视化界面 (11-20)

| 编号 | 功能 | 配置文件 |
|------|------|----------|
| 11 | 四鱼眼视图 | `four_fisheyes.rviz` |
| 12 | 左前鱼眼 | `fisheye_left.rviz` |
| 13 | 左上鱼眼 | `fisheye_left2.rviz` |
| 14 | 右前鱼眼 | `fisheye_right.rviz` |
| 15 | 右上鱼眼 | `fisheye_right2.rviz` |
| 16 | RGBD相机 | `rgbd_camera.rviz` |
| 17 | RGB相机 | `rgb_camera.rviz` |
| 18 | TOF传感器 | `tof.rviz` |
| 19 | SLAM可视化 | `slam_pose_markers.rviz` |
| 20 | 整体可视化 | `general.rviz` |

## 💡 高级使用技巧

### 批量执行

脚本支持多种输入格式：

```bash
# 单个编号
1

# 逗号分隔
1,3,5

# 范围（支持正序和倒序）
1-5
5-1

# 混合使用
1,3-5,10
```

### 并行监控

- 可以同时启动多个监控窗口
- 每个功能在新终端窗口中运行
- 支持同时监控多个传感器

### 环境自动配置

脚本会自动加载以下ROS2环境：
- `/opt/ros/galactic/setup.bash`
- `/opt/ros/humble/setup.bash`
- `~/ros2_ws/install/setup.bash`
- `~/colcon_ws/install/setup.bash`

## 📝 话题对照

### ROS1 Noetic vs ROS2 Galactic

| 功能 | ROS1话题 | ROS2话题 |
|------|----------|----------|
| 话题前缀 | `/xv_sdk/<序列号>/` | `/xv_sdk/<SN序列号>/` |
| 序列号格式 | `250801DR48FP25002587` | `SN250801DR48FP25002587` |
| 位姿 | `slam/pose` | `pose` |
| RGB相机 | `color_camera/image` | `rgb/image` |
| 鱼眼相机 | `fisheye_cameras/left/` | `fisheye_cameras_left/` |
| TOF深度 | `tof_camera/image` | `tof/depth/image_rect_raw` |
| 夹具 | `clamp/Data` | `clamp` |

### 自定义消息类型差异

| 消息 | ROS1 (xv_sdk) | ROS2 (xv_ros2_msgs) |
|------|---------------|---------------------|
| 位姿 | `PoseStampedConfidence` (自定义) | `geometry_msgs/PoseStamped` (标准) |
| 夹具 | `Clamp` | `Clamp` |
| 方向 | `OrientationStamped` | `OrientationStamped` |

**注意**：ROS2版本中的位姿话题直接使用标准`geometry_msgs/PoseStamped`，不包含ROS1版本中的`confidence`字段。

## 📝 指标参考

1. **pose**：`ros2 topic hz /xv_sdk/<SN序列号>/pose` 标准频率为500Hz
2. **rgb/image**：`ros2 topic hz /xv_sdk/<SN序列号>/rgb/image` 标准频率为60Hz
3. **tof/depth/image_rect_raw**：`ros2 topic hz /xv_sdk/<SN序列号>/tof/depth/image_rect_raw` 标准频率为30Hz
4. **clamp**：`ros2 topic echo /xv_sdk/<SN序列号>/clamp` 标准值范围为0～88

例如：`ros2 topic hz /xv_sdk/SN250801DR48FP25002587/pose`
5. **rviz2**：查看各位置摄像头图像是否清晰无明显卡顿

## ⚠️ 注意事项

1. **设备连接**：确保XV SDK设备已正确连接
2. **ROS2环境**：确保ROS2 Galactic环境已正确配置
3. **权限问题**：确保脚本有执行权限
4. **终端窗口**：新开的终端窗口可以通过Ctrl-C停止或直接关闭
5. **配置文件**：RViz2配置文件会根据设备序列号自动生成

## 🛠️ 故障排除

### 常见问题

#### 1. 找不到终端程序
```bash
# 安装gnome-terminal
sudo apt install gnome-terminal

# 或安装其他终端
sudo apt install konsole
sudo apt install xfce4-terminal
sudo apt install tilix
```

#### 2. ROS2环境未加载
```bash
# 手动source环境
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash

# 检查ROS2环境
echo $ROS_DISTRO
ros2 topic list
```

#### 3. 设备ID不匹配
```bash
# 检查设备连接状态
lsusb | grep -i xv

# 查看实际话题名称
ros2 topic list | grep xv_sdk
```

#### 4. RViz2配置文件不存在
```bash
# 手动生成配置文件（序列号带SN前缀）
bash rviz/scripts/generate_configs.sh SN250801DR48FP25002587

# 检查配置文件路径
ls -la rviz/generated/<设备序列号>/
```

#### 5. 权限问题
```bash
# 给脚本添加执行权限
chmod +x *.sh
chmod +x rviz/scripts/*.sh
```

## 🎯 最佳实践

1. **首次使用**：建议先运行单个功能测试
2. **性能监控**：使用频率监控功能检查数据流
3. **可视化调试**：结合RViz2视图进行视觉调试
4. **批量操作**：合理使用范围输入提高效率
5. **设备调试**：先监控频率，再查看数据内容
6. **多窗口管理**：合理使用多个终端窗口进行并行监控

## 📁 目录结构

```
ros2_galactic/
├── fastumi_monitor_menu.sh      # 主菜单脚本（多设备）
├── single_fastumi_monitor_menu.sh  # 单设备监控脚本
├── run_ros2topic_menu.sh        # 交互式菜单脚本
├── pose_to_markers.py           # Pose转Marker可视化节点
├── requirements.txt             # Python依赖
├── README.md                    # 本说明文档
├── topic/
│   └── galactic_interface.md    # ROS2话题接口说明
└── rviz/
    ├── templates/               # RViz2配置模板
    │   ├── fisheye_left.rviz.template
    │   ├── fisheye_left2.rviz.template
    │   ├── fisheye_right.rviz.template
    │   ├── fisheye_right2.rviz.template
    │   ├── four_fisheyes.rviz.template
    │   ├── general.rviz.template
    │   ├── rgb_camera.rviz.template
    │   ├── rgbd_camera.rviz.template
    │   ├── slam_pose_markers.rviz.template
    │   ├── slam_visualization.rviz.template
    │   └── tof.rviz.template
    ├── scripts/
    │   ├── generate_configs.sh  # 配置生成脚本
    │   └── device_manager.sh    # 设备管理脚本
    └── generated/               # 生成的配置文件目录
        └── <设备序列号>/
            └── *.rviz
```

---

**提示**：新开的终端窗口中按 Ctrl-C 可停止，或直接关闭窗口。

