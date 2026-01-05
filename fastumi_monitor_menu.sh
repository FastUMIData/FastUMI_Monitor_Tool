#!/usr/bin/env bash
# ROS2 Galactic版本 - XV SDK设备监控主菜单
# 自动检测所有连接的XV SDK设备并为每个设备打开监控终端

# 使用ros2 topic list获取设备列表
# ROS2中序列号格式为: SN250801DR48FP25002568 (带SN前缀)
XVISIO_SERIALS=$(ros2 topic list 2>/dev/null | grep -o '/xv_sdk/SN[^/]*' | cut -d'/' -f3 | sort -u)
echo "检测到的设备序列号: $XVISIO_SERIALS"

# 如果没有检测到设备
if [ -z "$XVISIO_SERIALS" ]; then
    echo "未检测到XV SDK设备，请确保:"
    echo "1. 已启动xv_sdk_ros2节点: ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py"
    echo "2. 设备已正确连接"
    exit 1
fi

mapfile -t XVISIO_SERIAL_ARRAY <<< "$XVISIO_SERIALS"

# 遍历数组，为每个设备打开监控窗口
for item in ${XVISIO_SERIAL_ARRAY[@]}; do
    echo "当前序列号: $item"
    gnome-terminal -- bash -c "bash single_fastumi_monitor_menu.sh $item"
done
