#!/usr/bin/env bash

# ROS2 Galactic版本 - 交互式监控菜单
# 简要说明：
# - 运行本脚本后，输入编号（支持单个、逗号列表、范围），将在新终端窗口中执行对应命令。
# - 你可以在新开的终端中按 Ctrl-C 停止，或直接关闭窗口。
# - 脚本会自动尝试加载ROS2环境
# 序列号格式: SN250801DR48FP25002568 (带SN前缀)

set -euo pipefail

# 主题前缀（根据你的实际设备ID）
# ROS2中序列号已包含SN前缀，如: SN250801DR48FP25002568
SERIAL_NUMBER=$1
TOPIC_PREFIX="/xv_sdk/${SERIAL_NUMBER}"
CURRENT_PATH=$(pwd)
RVIZ_PATH="${CURRENT_PATH}/rviz/generated/${SERIAL_NUMBER}"

# 构造需要执行的命令映射
# ROS2话题结构与ROS1有差异:
# - fisheye_cameras/left/ -> fisheye_cameras_left/
# - color_camera/ -> rgb/
# - tof_camera/ -> tof/depth/
# - slam/pose -> pose
# - clamp/Data -> clamp
declare -A CMD_MAP
CMD_MAP[1]="ros2 topic hz ${TOPIC_PREFIX}/imu"
CMD_MAP[2]="ros2 topic echo ${TOPIC_PREFIX}/imu"
CMD_MAP[3]="ros2 topic hz ${TOPIC_PREFIX}/pose"
CMD_MAP[4]="ros2 topic echo ${TOPIC_PREFIX}/pose"
CMD_MAP[5]="ros2 topic hz ${TOPIC_PREFIX}/rgb/image"
CMD_MAP[6]="ros2 topic hz ${TOPIC_PREFIX}/fisheye_cameras_left/camera_info"
CMD_MAP[7]="ros2 topic hz ${TOPIC_PREFIX}/fisheye_cameras_left2/camera_info"
CMD_MAP[8]="ros2 topic hz ${TOPIC_PREFIX}/fisheye_cameras_right/camera_info"
CMD_MAP[9]="ros2 topic hz ${TOPIC_PREFIX}/fisheye_cameras_right2/camera_info"
CMD_MAP[10]="ros2 topic hz ${TOPIC_PREFIX}/tof/depth/image_rect_raw"
CMD_MAP[11]="rviz2 -d ${RVIZ_PATH}/four_fisheyes.rviz"
CMD_MAP[12]="rviz2 -d ${RVIZ_PATH}/fisheye_left.rviz"
CMD_MAP[13]="rviz2 -d ${RVIZ_PATH}/fisheye_left2.rviz"
CMD_MAP[14]="rviz2 -d ${RVIZ_PATH}/fisheye_right.rviz"
CMD_MAP[15]="rviz2 -d ${RVIZ_PATH}/fisheye_right2.rviz"
CMD_MAP[16]="rviz2 -d ${RVIZ_PATH}/rgbd_camera.rviz"
CMD_MAP[17]="rviz2 -d ${RVIZ_PATH}/rgb_camera.rviz"
CMD_MAP[18]="rviz2 -d ${RVIZ_PATH}/tof.rviz"
CMD_MAP[19]="rviz2 -d ${RVIZ_PATH}/slam_pose_markers.rviz"
CMD_MAP[20]="rviz2 -d ${RVIZ_PATH}/general.rviz"
CMD_MAP[21]="ros2 service call /xv_sdk/${SERIAL_NUMBER}/stop_clamp std_srvs/srv/Trigger;ros2 service call /xv_sdk/${SERIAL_NUMBER}/start_clamp std_srvs/srv/Trigger;ros2 topic echo ${TOPIC_PREFIX}/clamp"
# ROS1: rosservice call ${TOPIC_PREFIX}/clamp/stop;rosservice call ${TOPIC_PREFIX}/clamp/start;

# 终端自动探测顺序
TERM_CANDIDATES=(
  "gnome-terminal"
  "konsole"
  "xfce4-terminal"
  "tilix"
  "xterm"
)

detect_terminal() {
  for t in "${TERM_CANDIDATES[@]}"; do
    if command -v "$t" >/dev/null 2>&1; then
      echo "$t"
      return 0
    fi
  done
  return 1
}

# 生成通用命令：加载 ROS2 环境 + 目标命令
build_shell_command() {
  local user_cmd="$1"
  # 按需加载常见 ROS2 环境
  local src_cmd=""
  src_cmd+="if [ -f /opt/ros/galactic/setup.bash ]; then source /opt/ros/galactic/setup.bash; fi;"
  src_cmd+="if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi;"
  src_cmd+="if [ -f \"\$HOME/ros2_ws/install/setup.bash\" ]; then source \"\$HOME/ros2_ws/install/setup.bash\"; fi;"
  src_cmd+="if [ -f \"\$HOME/colcon_ws/install/setup.bash\" ]; then source \"\$HOME/colcon_ws/install/setup.bash\"; fi;"

  # 使用登录 shell 以便环境变量完整生效；末尾 exec bash 便于命令退出后仍停留
  echo "${src_cmd} ${user_cmd}; exec bash"
}

open_in_terminal() {
  local title="$1"
  local user_cmd="$2"
  local term_bin
  term_bin=$(detect_terminal) || {
    echo "未找到可用的终端程序（尝试：${TERM_CANDIDATES[*]}）。请安装其中之一。" >&2
    exit 1
  }

  local shell_cmd
  shell_cmd=$(build_shell_command "$user_cmd")

  case "$term_bin" in
    gnome-terminal)
      nohup gnome-terminal --title="$title" -- bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    konsole)
      nohup konsole -p tabtitle="$title" -e bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    xfce4-terminal)
      nohup xfce4-terminal --title="$title" --hold -x bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    tilix)
      nohup tilix -t "$title" -e bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    xterm)
      nohup xterm -T "$title" -e bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    *)
      echo "未知终端：$term_bin" >&2
      exit 1
      ;;
  esac
}

parse_input_to_ids() {
  # 将原始输入（支持：单个数字、逗号列表、范围 1-4、以及混合）解析为去重后的编号数组
  local raw="$1"
  SELECTED_IDS=()
  declare -A seen

  local parts=()
  IFS=',' read -ra parts <<< "$raw"

  for part in "${parts[@]}"; do
    # 去除空白
    part="${part//[[:space:]]/}"
    [[ -z "$part" ]] && continue

    if [[ "$part" =~ ^[0-9]+-[0-9]+$ ]]; then
      local a b
      IFS='-' read -r a b <<< "$part"
      # 允许反序范围，如 4-1
      if (( a > b )); then
        local tmp=$a; a=$b; b=$tmp
      fi
      for ((i=a; i<=b; i++)); do
        if [[ -n "${CMD_MAP[$i]+x}" && -z "${seen[$i]+x}" ]]; then
          SELECTED_IDS+=("$i")
          seen[$i]=1
        fi
      done
    elif [[ "$part" =~ ^[0-9]+$ ]]; then
      local i=$part
      if [[ -n "${CMD_MAP[$i]+x}" && -z "${seen[$i]+x}" ]]; then
        SELECTED_IDS+=("$i")
        seen[$i]=1
      fi
    else
      echo "跳过无效输入片段：$part" >&2
    fi
  done
}

print_menu() {
  cat <<EOF
====== ROS2 Galactic XV SDK 监控菜单 (设备: ${SERIAL_NUMBER}) ======
选择要运行的命令（支持单个编号、逗号列表、范围）
 1) imu 频率       hz   imu
 2) imu 数据读取   echo imu
 3) SLAM 频率       hz   pose
 4) SLAM 数据读取   echo pose
 5) RGB相机频率     hz   rgb/image # 如果无输出，可使用rqt先查看后再运行5
 6) 左前鱼眼相机频率  hz   fisheye_cameras_left/camera_info
 7) 左上鱼眼相机频率  hz   fisheye_cameras_left2/camera_info
 8) 右前鱼眼相机频率  hz   fisheye_cameras_right/camera_info
 9) 右上鱼眼相机频率  hz   fisheye_cameras_right2/camera_info
10) TOF相机频率     hz   tof/depth/image_rect_raw
11) RViz2: 四鱼眼视图            four_fisheyes.rviz
12) RViz2: 左前鱼眼              fisheye_left.rviz
13) RViz2: 左上鱼眼              fisheye_left2.rviz
14) RViz2: 右前鱼眼              fisheye_right.rviz
15) RViz2: 右上鱼眼              fisheye_right2.rviz
16) RViz2: RGBD 相机             rgbd_camera.rviz
17) RViz2: RGB 相机              rgb_camera.rviz
18) RViz2: TOF                   tof.rviz
19) RViz2: SLAM Pose标记         slam_pose_markers.rviz
20) RViz2: 整体可视化            general.rviz
21) CLAMP 数据读取   echo clamp
 0) 退出脚本
==============================================================
提示：新开的终端窗口中按 Ctrl-C 可停止，或直接关闭窗口。
EOF
}

main() {
  while true; do
    print_menu
    read -rp "请输入编号(0-21)，可用如 1-4 或 1,2,3: " user_input
    if [[ -z "${user_input:-}" ]]; then
      echo "未输入编号，继续等待..."
      continue
    fi
    if [[ "$user_input" == "0" ]]; then
      echo "已退出脚本。"
      # 如果输入编号为0,先执行ros2 service call /xv_sdk/SN/stop_clamp std_srvs/srv/Trigger，即关闭夹具数据输出
      ros2 service call /xv_sdk/${SERIAL_NUMBER}/stop_clamp std_srvs/srv/Trigger
      exit 0
    fi
    parse_input_to_ids "$user_input"
    if (( ${#SELECTED_IDS[@]} == 0 )); then
      echo "没有可执行的有效编号（范围：1-21）。"
      continue
    fi

    for num in "${SELECTED_IDS[@]}"; do
      local cmd_title="[#${num}] ${CMD_MAP[$num]}"
      open_in_terminal "$cmd_title" "${CMD_MAP[$num]}"
      echo "已在新终端窗口启动：${CMD_MAP[$num]}"
    done
  done
}

main "$@"
