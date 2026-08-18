#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

set -eo pipefail

WS="$HOME/ros2_ws"
RUN_DIR="$WS/left_probe_service/run"

count_field()
{
    local topic="$1"
    local field="$2"

    ros2 topic info "$topic" 2>/dev/null |
    awk -v field="$field" '
        index($0, field) {
            print $3
            exit
        }
    '
}

echo "========================================================================"
echo "左臂探头常驻服务状态"
echo "========================================================================"

echo
echo "左臂CANFD订阅者：$(
    count_field \
        /left_arm/rm_driver/movej_canfd_cmd \
        "Subscription count:"
)"

echo "D435深度图发布者：$(
    count_field \
        /left_probe/d435/depth/image_rect_raw \
        "Publisher count:"
)"

echo "测距话题发布者：$(
    count_field \
        /left_probe/near_distance_m \
        "Publisher count:"
)"

echo
echo "任务服务："

ros2 service list 2>/dev/null |
grep -E '^/left_probe/task/' || true

echo
echo "当前任务状态："

if ros2 service list 2>/dev/null |
   grep -qx "/left_probe/task/status"; then

    timeout 5 ros2 service call \
        /left_probe/task/status \
        std_srvs/srv/Trigger \
        "{}" || true
else
    echo "[OFFLINE] 任务接口未运行"
fi

echo
echo "当前D435距离："

timeout 3 ros2 topic echo \
    /left_probe/near_distance_m \
    std_msgs/msg/Float32 \
    --once 2>/dev/null || echo "无有效距离"

echo
echo "PID文件："

for name in driver camera distance task_server; do
    file="$RUN_DIR/$name.pid"

    if [[ -f "$file" ]]; then
        pid="$(cat "$file")"

        if kill -0 "$pid" 2>/dev/null; then
            echo "$name: PID=$pid RUNNING"
        else
            echo "$name: PID=$pid STALE"
        fi
    else
        echo "$name: 无PID文件（未启动或复用外部节点）"
    fi
done

if [[ -f "$RUN_DIR/current_log_dir" ]]; then
    echo
    echo "基础服务日志：$(cat "$RUN_DIR/current_log_dir")"
fi
