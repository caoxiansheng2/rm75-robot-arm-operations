#!/usr/bin/env bash
set -e

WS="$HOME/ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "========================================================================"
echo "左右臂好管家动作系统"
echo "========================================================================"
echo
echo "左臂：启动常驻 driver + D435 + distance + task server"
echo "右臂：收到 button 后由 right_arm_press_button.sh 按需启动/复用"
echo "HTTP：http://127.0.0.1:8080"
echo

"$WS/start_left_probe_service.sh"

echo
echo "========================================================================"
echo "[READY] 左右臂好管家动作入口已就绪"
echo "========================================================================"
echo
echo "好管家："
echo "  detection -> 左臂局放"
echo "  button    -> 右臂QR/按钮动作"
echo "  task_three -> 右臂旋钮动作"
echo
