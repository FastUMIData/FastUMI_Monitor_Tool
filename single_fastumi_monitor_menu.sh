#!/usr/bin/env bash
# ROS2 Galactic版本 - 单设备监控菜单
# 用法: bash single_fastumi_monitor_menu.sh <设备序列号>
# 序列号格式: SN250801DR48FP25002568 (带SN前缀)

XVISIO_SERIALS="$1"

if [ -z "$XVISIO_SERIALS" ]; then
    echo "用法: bash single_fastumi_monitor_menu.sh <设备序列号>"
    echo "示例: bash single_fastumi_monitor_menu.sh SN250801DR48FP25002587"
    echo "注意: serial number should start with 'SN'"
    exit 1
fi

# 验证序列号格式
if [[ ! "$XVISIO_SERIALS" =~ ^SN ]]; then
    echo "Warning: Serial number should start with SN, e.g.: SN250801DR48FP25002587"
    echo "Received serial number: $XVISIO_SERIALS"
fi

echo "Initializing device: $XVISIO_SERIALS"

# 生成RViz配置文件
bash rviz/scripts/generate_configs.sh $XVISIO_SERIALS
echo "RViz config generated for device: $XVISIO_SERIALS"

# 安装Python依赖
echo "Installing dependencies..."
conda run -n fastumi pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
echo "Dependencies installed"

# 启动pose_to_markers节点（后台运行）
conda run -n fastumi python3 pose_to_markers.py $XVISIO_SERIALS &
echo "waiting for pose_to_markers.py to start..."
sleep 2

# 打开交互式菜单
echo "opening monitor menu..."
bash run_ros2topic_menu.sh $XVISIO_SERIALS
