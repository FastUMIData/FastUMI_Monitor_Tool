#!/usr/bin/env bash
# ROS2 Galactic版本 - 单设备监控菜单
# 用法: bash single_fastumi_monitor_menu.sh <设备序列号>
# 序列号格式: SN250801DR48FP25002568 (带SN前缀)

XVISIO_SERIALS="$1"

if [ -z "$XVISIO_SERIALS" ]; then
    echo "用法: bash single_fastumi_monitor_menu.sh <设备序列号>"
    echo "示例: bash single_fastumi_monitor_menu.sh SN250801DR48FP25002587"
    echo "注意: 序列号需要带SN前缀"
    exit 1
fi

# 验证序列号格式
if [[ ! "$XVISIO_SERIALS" =~ ^SN ]]; then
    echo "警告: 序列号应以SN开头，如: SN250801DR48FP25002587"
    echo "收到的序列号: $XVISIO_SERIALS"
fi

echo "初始化设备: $XVISIO_SERIALS"

# 生成RViz配置文件
bash rviz/scripts/generate_configs.sh $XVISIO_SERIALS
echo "RViz配置文件生成成功"

# 安装Python依赖
echo "安装依赖..."
conda run -n fastumi pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
echo "依赖安装完成"

# 启动pose_to_markers节点（后台运行）
conda run -n fastumi python3 pose_to_markers.py $XVISIO_SERIALS &
echo "等待 pose_to_markers.py 启动..."
sleep 2

# 打开交互式菜单
echo "打开监控菜单"
bash run_ros2topic_menu.sh $XVISIO_SERIALS
