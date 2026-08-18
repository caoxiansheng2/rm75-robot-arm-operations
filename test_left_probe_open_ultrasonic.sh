#!/usr/bin/env bash
set -euo pipefail

MODE_FILE="$HOME/ros2_ws/left_probe_service/run/next_detection_type"

mkdir -p "$(dirname "$MODE_FILE")"

printf '%s\n' \
  "open_ultrasonic" \
  > "$MODE_FILE"

echo "============================================================"
echo "下一次好管家 detection 已切换为单通道演示"
echo "检测方式：开放式超声波 open_ultrasonic / 0x01"
echo "只对下一次 detection 生效，执行后自动恢复四通道 all"
echo
echo "现在请在好管家发送 detection。"
echo "============================================================"
