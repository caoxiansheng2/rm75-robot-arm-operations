#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

set -eo pipefail

ACTION="${1:-status}"

case "$ACTION" in
    start|execute)
        SERVICE="/left_probe/task/start"
        ;;

    plan)
        SERVICE="/left_probe/task/plan"
        ;;

    reset)
        SERVICE="/left_probe/task/reset"
        ;;

    status)
        SERVICE="/left_probe/task/status"
        ;;

    cancel|stop)
        SERVICE="/left_probe/task/cancel"
        ;;

    *)
        echo "用法："
        echo "  $0 start    # 异步执行完整探头任务"
        echo "  $0 plan     # 异步进行复位和轨迹规划"
        echo "  $0 reset    # 异步复位到初始位置"
        echo "  $0 status   # 查询当前任务状态"
        echo "  $0 cancel   # 停止当前任务"
        exit 2
        ;;
esac

if ! ros2 service list 2>/dev/null |
   grep -qx "$SERVICE"; then

    echo "[ERROR] 服务不存在：$SERVICE"
    echo "先启动："
    echo "  ~/ros2_ws/start_left_probe_service.sh"
    exit 1
fi

ros2 service call \
    "$SERVICE" \
    std_srvs/srv/Trigger \
    "{}"
