#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

set -eo pipefail

WS="$HOME/ros2_ws"
SERVICE_DIR="$WS/left_probe_service"
RUN_DIR="$SERVICE_DIR/run"
LOG_ROOT="$SERVICE_DIR/logs"
LOG_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)"

POSE_FILE="$WS/left_arm_longstroke_start_retracted_150mm.json"
INITIAL_SCRIPT="$WS/left_arm_ensure_initial_pose.py"
CAMERA_SCRIPT="$WS/start_left_d435.sh"
DISTANCE_SCRIPT="$WS/left_d435_distance.py"
TASK_SERVER="$WS/left_probe_task_server.py"
HTTP_SERVER="$WS/robot_action_http_server.py"

HTTP_HOST="127.0.0.1"
HTTP_PORT="8080"

PROFILE="424x240x30"

mkdir -p "$RUN_DIR" "$LOG_DIR"
echo "$LOG_DIR" > "$RUN_DIR/current_log_dir"

STARTED_NAMES=()
STARTED_PIDS=()
START_SUCCESS="false"

topic_sub_count()
{
    ros2 topic info "$1" 2>/dev/null |
    awk '/Subscription count:/ {print $3}'
}

topic_pub_count()
{
    ros2 topic info "$1" 2>/dev/null |
    awk '/Publisher count:/ {print $3}'
}

positive_integer()
{
    [[ "${1:-}" =~ ^[0-9]+$ ]] && (( "$1" >= 1 ))
}

driver_ready()
{
    local canfd state movej

    canfd="$(topic_sub_count \
        /left_arm/rm_driver/movej_canfd_cmd)"

    state="$(topic_sub_count \
        /left_arm/rm_driver/get_current_arm_state_cmd)"

    movej="$(topic_sub_count \
        /left_arm/rm_driver/movej_cmd)"

    positive_integer "$canfd" &&
    positive_integer "$state" &&
    positive_integer "$movej"
}

camera_ready()
{
    local count
    count="$(topic_pub_count \
        /left_probe/d435/depth/image_rect_raw)"
    positive_integer "$count"
}

distance_ready()
{
    local count
    count="$(topic_pub_count \
        /left_probe/near_distance_m)"
    positive_integer "$count"
}

task_server_ready()
{
    ros2 service list 2>/dev/null |
    grep -qx "/left_probe/task/start"
}

http_ready()
{
    python3 - "$HTTP_PORT" <<'PYHTTP'
import sys
import urllib.error
import urllib.request

port = int(sys.argv[1])

url = (
    f"http://127.0.0.1:"
    f"{port}/health"
)

try:
    with urllib.request.urlopen(
        url,
        timeout=0.5,
    ) as response:
        sys.exit(
            0
            if response.status in (200, 503)
            else 1
        )

except urllib.error.HTTPError as exc:
    # HTTP服务器已起来但health返回503，
    # 仍然说明监听端口已经正常建立。
    sys.exit(
        0
        if exc.code in (200, 503)
        else 1
    )

except Exception:
    sys.exit(1)
PYHTTP
}

register_started()
{
    local name="$1"
    local pid="$2"

    STARTED_NAMES+=("$name")
    STARTED_PIDS+=("$pid")

    echo "$pid" > "$RUN_DIR/$name.pid"
}

stop_group()
{
    local pid="$1"
    local name="$2"

    if [[ -z "$pid" ]] ||
       ! kill -0 "$pid" 2>/dev/null; then
        return
    fi

    echo "[cleanup] 停止$name，PID=$pid"

    kill -INT -- "-$pid" 2>/dev/null || true

    for _ in $(seq 1 30); do
        if ! kill -0 "$pid" 2>/dev/null; then
            return
        fi
        sleep 0.1
    done

    kill -TERM -- "-$pid" 2>/dev/null || true
}

cleanup_on_failure()
{
    local rc=$?

    trap - EXIT

    if [[ "$START_SUCCESS" != "true" ]]; then
        echo
        echo "[ERROR] 常驻服务启动失败，清理本次启动的进程"

        for ((i=${#STARTED_PIDS[@]}-1; i>=0; i--)); do
            stop_group \
                "${STARTED_PIDS[$i]}" \
                "${STARTED_NAMES[$i]}"
        done
    fi

    exit "$rc"
}

trap cleanup_on_failure EXIT

wait_condition()
{
    local description="$1"
    local timeout="$2"
    shift 2

    echo "[wait] $description"

    for _ in $(seq 1 "$timeout"); do
        if "$@"; then
            echo "[PASS] $description"
            return 0
        fi

        sleep 0.5
    done

    echo "[FAIL] 等待超时：$description"
    return 1
}

wait_valid_depth()
{
    for _ in $(seq 1 40); do
        output="$(
            timeout 2 ros2 topic echo \
                /left_probe/distance_valid \
                std_msgs/msg/Bool \
                --once 2>/dev/null || true
        )"

        if grep -q "data: true" <<< "$output"; then
            echo "[PASS] D435距离有效"
            return 0
        fi

        sleep 0.25
    done

    echo "[FAIL] D435没有产生有效距离"
    return 1
}

require_file()
{
    if [[ ! -f "$1" ]]; then
        echo "[FAIL] 缺少文件：$1"
        exit 1
    fi
}

echo "========================================================================"
echo "启动左臂探头常驻服务（暂不包含G300）"
echo "========================================================================"
echo "日志目录：$LOG_DIR"

if task_server_ready && http_ready; then
    echo "[READY] 常驻任务服务和HTTP接口已经运行"
    echo
    echo "查看状态："
    echo "  $WS/status_left_probe_service.sh"
    exit 0
fi

require_file "$POSE_FILE"
require_file "$INITIAL_SCRIPT"
require_file "$CAMERA_SCRIPT"
require_file "$DISTANCE_SCRIPT"
require_file "$TASK_SERVER"
require_file "$HTTP_SERVER"

python3 -m py_compile \
    "$INITIAL_SCRIPT" \
    "$DISTANCE_SCRIPT" \
    "$TASK_SERVER" \
    "$HTTP_SERVER" \
    "$WS/left_arm_depth_cycle.py" \
    "$WS/g300_full_acquire.py"

echo
echo "========================================================================"
echo "1. 启动或复用左臂驱动"
echo "========================================================================"

if driver_ready; then
    echo "[REUSE] 左臂rm_driver已经运行"
else
    setsid ros2 launch \
        rm_driver \
        rm_75_left_driver.launch.py \
        >"$LOG_DIR/left_arm_driver.log" 2>&1 &

    DRIVER_PID=$!
    register_started "driver" "$DRIVER_PID"

    echo "左臂驱动PID：$DRIVER_PID"

    wait_condition \
        "左臂驱动控制接口" \
        80 \
        driver_ready || {
            tail -n 100 "$LOG_DIR/left_arm_driver.log" || true
            exit 1
        }
fi

echo
echo "========================================================================"
echo "2. 检查并直接复位机械臂初始位姿"
echo "========================================================================"

set +e

python3 -u "$INITIAL_SCRIPT" \
    --pose-file "$POSE_FILE" \
    --move \
    --speed 5 \
    --timeout 60 \
    --joint-tolerance-deg 0.50 \
    --verify-tolerance-deg 0.60 \
    2>&1 |
tee "$LOG_DIR/initial_pose_startup.log"

INITIAL_STATUS=${PIPESTATUS[0]}

set -e

if (( INITIAL_STATUS != 0 )); then
    echo "[FAIL] 初始位姿复位失败"
    exit "$INITIAL_STATUS"
fi

echo
echo "========================================================================"
echo "3. 启动或复用左侧D435"
echo "========================================================================"

if camera_ready; then
    echo "[REUSE] 左侧D435已经运行"
else
    setsid "$CAMERA_SCRIPT" "$PROFILE" \
        >"$LOG_DIR/camera.log" 2>&1 &

    CAMERA_PID=$!
    register_started "camera" "$CAMERA_PID"

    echo "D435 PID：$CAMERA_PID"

    wait_condition \
        "D435深度图话题" \
        50 \
        camera_ready || {
            tail -n 100 "$LOG_DIR/camera.log" || true
            exit 1
        }
fi

echo
echo "========================================================================"
echo "4. 启动或复用深度测距节点"
echo "========================================================================"

if distance_ready; then
    echo "[REUSE] 深度测距节点已经运行"
else
    setsid python3 -u "$DISTANCE_SCRIPT" \
        --roi-width 30 \
        --roi-height 30 \
        --filter-frames 3 \
        --min-valid-ratio 0.50 \
        >"$LOG_DIR/distance.log" 2>&1 &

    DISTANCE_PID=$!
    register_started "distance" "$DISTANCE_PID"

    echo "测距节点PID：$DISTANCE_PID"

    wait_condition \
        "D435近距离测量话题" \
        40 \
        distance_ready || {
            tail -n 100 "$LOG_DIR/distance.log" || true
            exit 1
        }
fi

wait_valid_depth || {
    tail -n 100 "$LOG_DIR/distance.log" || true
    exit 1
}

echo
echo "========================================================================"
echo "5. 启动或复用常驻任务接口"
echo "========================================================================"

if task_server_ready; then
    echo "[REUSE] 任务服务器已经运行"
else
    setsid python3 -u "$TASK_SERVER" \
        --workspace "$WS" \
        --pose-file "$POSE_FILE" \
        --restore-speed 5 \
        --restore-timeout 60 \
        --stop-depth 0.140 \
        --hard-min-depth 0.090 \
        --slow-depth 0.200 \
        --max-distance 0.650 \
        --reach-margin 0.010 \
        --depth-motion-scale 0.80 \
        --fast-speed 0.080 \
        --slow-speed 0.015 \
        --return-speed 0.080 \
        --dwell 2.0 \
        --rate 100 \
        --key-step 0.0005 \
        >"$LOG_DIR/task_server.log" 2>&1 &

    TASK_PID=$!
    register_started \
        "task_server" \
        "$TASK_PID"

    echo "任务接口PID：$TASK_PID"

    wait_condition \
        "任务调用接口" \
        40 \
        task_server_ready || {
            tail -n 100 \
                "$LOG_DIR/task_server.log" \
                || true
            exit 1
        }
fi


echo
echo "========================================================================"
echo "6. 启动或复用HTTP接口"
echo "========================================================================"

if http_ready; then
    echo "[REUSE] HTTP接口已经运行"
else
    setsid python3 -u "$HTTP_SERVER" \
        --host "$HTTP_HOST" \
        --port "$HTTP_PORT" \
        >"$LOG_DIR/http_server.log" 2>&1 &

    HTTP_PID=$!

    register_started \
        "http_server" \
        "$HTTP_PID"

    echo "HTTP接口PID：$HTTP_PID"

    wait_condition \
        "HTTP接口 http://127.0.0.1:$HTTP_PORT" \
        30 \
        http_ready || {
            tail -n 100 \
                "$LOG_DIR/http_server.log" \
                || true
            exit 1
        }
fi


START_SUCCESS="true"

echo
echo "========================================================================"
echo "[READY] 左臂探头常驻服务已启动"
echo "[READY] HTTP接口：http://127.0.0.1:$HTTP_PORT"
echo "========================================================================"
echo
echo "启动完整任务："
echo "  $WS/call_left_probe_task.sh start"
echo
echo "只做规划："
echo "  $WS/call_left_probe_task.sh plan"
echo
echo "复位初始位置："
echo "  $WS/call_left_probe_task.sh reset"
echo
echo "查看状态："
echo "  $WS/call_left_probe_task.sh status"
echo
echo "取消任务："
echo "  $WS/call_left_probe_task.sh cancel"
echo
echo "停止全部常驻服务："
echo "  $WS/stop_left_probe_service.sh"
echo
echo "本次基础服务日志：$LOG_DIR"
