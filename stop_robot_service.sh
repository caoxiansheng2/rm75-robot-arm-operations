#!/usr/bin/env bash
set +e

WS="$HOME/ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "========================================================================"
echo "停止左右臂动作系统"
echo "========================================================================"

echo
echo "[0/3] 停止可能正在执行的右臂旋钮任务"

pkill -TERM -f   "$WS/right_arm_turn_knob.sh"   2>/dev/null || true

sleep 2

# 如果旋钮脚本异常退出而专用launch仍残留，再兜底。
pkill -INT -f   "ros2 launch rm75_fixed_grasp turn_knob.launch.py"   2>/dev/null || true

sleep 1

pkill -TERM -f   "ros2 launch rm75_fixed_grasp turn_knob.launch.py"   2>/dev/null || true

echo
echo "[1/2] 停止右臂本脚本管理的组件"

if [[ -x "$WS/right_arm_press_button.sh" ]]; then
    "$WS/right_arm_press_button.sh" stop || true
fi

echo
echo "[2/2] 停止左臂常驻服务"

"$WS/stop_left_probe_service.sh" || true

# 新统一HTTP的兜底。
pkill -TERM -f \
  "$WS/robot_action_http_server.py" \
  2>/dev/null || true

sleep 1

pkill -KILL -f \
  "$WS/robot_action_http_server.py" \
  2>/dev/null || true

echo
echo "========================================================================"
echo "[DONE] 左右臂动作系统已停止"
echo "========================================================================"
