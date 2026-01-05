# ROS2 Galactic Topic 接口说明

## 话题列表

以下是XV SDK在ROS2 Galactic中发布的话题列表（以设备SN250801DR48FP25002587为例）：

```
/parameter_events
/rosout
/tf_static
/xv_sdk/SN250801DR48FP25002587/clamp
/xv_sdk/SN250801DR48FP25002587/fisheye_cameras_left/camera_info
/xv_sdk/SN250801DR48FP25002587/fisheye_cameras_left/image
/xv_sdk/SN250801DR48FP25002587/fisheye_cameras_left2/camera_info
/xv_sdk/SN250801DR48FP25002587/fisheye_cameras_left2/image
/xv_sdk/SN250801DR48FP25002587/fisheye_cameras_right/camera_info
/xv_sdk/SN250801DR48FP25002587/fisheye_cameras_right/image
/xv_sdk/SN250801DR48FP25002587/fisheye_cameras_right2/camera_info
/xv_sdk/SN250801DR48FP25002587/fisheye_cameras_right2/image
/xv_sdk/SN250801DR48FP25002587/imu
/xv_sdk/SN250801DR48FP25002587/pose
/xv_sdk/SN250801DR48FP25002587/rgb/camera_info
/xv_sdk/SN250801DR48FP25002587/rgb/image
/xv_sdk/SN250801DR48FP25002587/rgbPointCloud
/xv_sdk/SN250801DR48FP25002587/rgbPointCloud/camera_info
/xv_sdk/SN250801DR48FP25002587/rgb_rectification/camera_info
/xv_sdk/SN250801DR48FP25002587/rgb_rectification/image
/xv_sdk/SN250801DR48FP25002587/rgbd/camera_info
/xv_sdk/SN250801DR48FP25002587/rgbd/image
/xv_sdk/SN250801DR48FP25002587/rgbd/raw/camera_info
/xv_sdk/SN250801DR48FP25002587/rgbd/raw/data
/xv_sdk/SN250801DR48FP25002587/rgbd/raw/image
/xv_sdk/SN250801DR48FP25002587/tof/depth/camera_info
/xv_sdk/SN250801DR48FP25002587/tof/depth/image_rect_raw
```

## 自定义消息类型对比

### ROS1 xv_sdk 自定义消息

| 消息名称 | 字段定义 |
|----------|----------|
| `PoseStampedConfidence` | `float64 confidence` + `geometry_msgs/PoseStamped poseMsg` |
| `Clamp` | `std_msgs/Header header` + `float64 data` + `float64 timestamp` |
| `OrientationStamped` | `header` + `float64[9] matrix` + `Quaternion` + `Vector3 angularVelocity` |
| `FisheyeImages` | 鱼眼图像集合 |
| `Lost` | 丢失状态 |
| `Plane/Planes` | 平面检测 |

### ROS2 xv_ros2_msgs 自定义消息

| 消息名称 | 字段定义 |
|----------|----------|
| `Clamp` | `std_msgs/Header header` + `float64 data` + `float64 timestamp` |
| `OrientationStamped` | `header` + `float64[9] matrix` + `Quaternion` + `Vector3 angular_velocity` |
| `Controller` | 控制器数据 |
| `EventData` | 事件数据 |
| `ButtonMsg` | 按钮消息 |
| `ColorDepth` | RGBD原始数据 |

### 关键差异

1. **PoseStampedConfidence**: ROS1中使用自定义消息，ROS2中直接使用标准`geometry_msgs/PoseStamped`
2. **OrientationStamped**: 字段名差异 - ROS1用`angularVelocity`，ROS2用`angular_velocity`
3. **clamp话题**: ROS1为`clamp/Data`，ROS2简化为`clamp`

## ROS1 vs ROS2 话题差异对照

| 功能 | ROS1 Noetic 话题 | ROS2 Galactic 话题 |
|------|------------------|-------------------|
| 话题前缀 | `/xv_sdk/<序列号>/` | `/xv_sdk/<SN序列号>/` |
| 序列号格式 | `250801DR48FP25002587` | `SN250801DR48FP25002587` |
| 位姿 | `slam/pose` | `pose` |
| 夹具 | `clamp/Data` | `clamp` |
| RGB相机 | `color_camera/image` | `rgb/image` |
| 左前鱼眼 | `fisheye_cameras/left/image` | `fisheye_cameras_left/image` |
| 左上鱼眼 | `fisheye_cameras/left2/image` | `fisheye_cameras_left2/image` |
| 右前鱼眼 | `fisheye_cameras/right/image` | `fisheye_cameras_right/image` |
| 右上鱼眼 | `fisheye_cameras/right2/image` | `fisheye_cameras_right2/image` |
| TOF深度 | `tof_camera/image` | `tof/depth/image_rect_raw` |
| RGBD | `rgbd_camera/image` | `rgbd/image` (RGB8编码) |
| IMU | `imu_sensor/data_raw` | `imu` |

## 详细话题说明

### /xv_sdk/sn/rgbd/image
- **描述**: RGBD相机图像消息
- **内容**: 包含转换后的RGBD相机的图像数据
- **编码**: RGB8
- **数据格式**: uint8

### /xv_sdk/sn/rgbd/camera_info
- **描述**: RGBD相机信息消息
- **内容**: 包含RGBD相机的时间戳，frame ID，宽高及标定参数

### /xv_sdk/sn/rgbd/raw/image
- **描述**: RGBD相机原始图像消息
- **内容**: 包含RGBD相机的原始图像数据
- **编码**: RGB8
- **数据格式**: uint8

### /xv_sdk/sn/rgbd/raw/camera_info
- **描述**: RGBD相机原始数据信息消息
- **内容**: 包含RGBD相机的时间戳，frame ID，宽高及标定参数

## 命令差异

| 功能 | ROS1 Noetic | ROS2 Galactic |
|------|-------------|---------------|
| 查看话题频率 | `rostopic hz` | `ros2 topic hz` |
| 查看话题内容 | `rostopic echo` | `ros2 topic echo` |
| 启动节点 | `roslaunch` | `ros2 launch` |
| 服务调用 | `rosservice call` | `ros2 service call` |
| 可视化工具 | `rviz` | `rviz2` |
| 话题列表 | `rostopic list` | `ros2 topic list` |

