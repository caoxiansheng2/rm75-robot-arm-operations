#!/usr/bin/env bash

# ===== LEFT_PROBE_HTTP_CLEANUP_BEGIN =====
cleanup_left_probe_http() {
    echo
    echo "========================================================================"
    echo "0. 清理左臂 HTTP 接口"
    echo "========================================================================"

    local ws="$HOME/ros2_ws"
    local run_dir="$ws/left_probe_service/run"
    local name
    local pid
    local cmd

    # ------------------------------------------------------------------
    # 1) 先根据脚本路径精确清理两种 HTTP 服务
    # ------------------------------------------------------------------
    for name in \
        left_probe_http_server.py \
        left_probe_haoguanjia_server.py
    do
        while read -r pid
        do
            [ -z "$pid" ] && continue
            [ ! -d "/proc/$pid" ] && continue

            cmd="$(
                tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null
            )"

            echo "[HTTP TERM] PID=$pid"
            echo "            $cmd"

            kill -TERM "$pid" 2>/dev/null || true

        done < <(
            pgrep -f "$ws/$name" 2>/dev/null || true
        )
    done

    sleep 1

    # ------------------------------------------------------------------
    # 2) TERM 后仍存活则 KILL
    # ------------------------------------------------------------------
    for name in \
        left_probe_http_server.py \
        left_probe_haoguanjia_server.py
    do
        while read -r pid
        do
            [ -z "$pid" ] && continue
            [ ! -d "/proc/$pid" ] && continue

            echo "[HTTP KILL] PID=$pid"

            kill -KILL "$pid" 2>/dev/null || true

        done < <(
            pgrep -f "$ws/$name" 2>/dev/null || true
        )
    done

    # ------------------------------------------------------------------
    # 3) 清理旧 PID 文件
    # ------------------------------------------------------------------
    rm -f \
        "$run_dir/http_server.pid" \
        2>/dev/null || true

    sleep 0.3

    # ------------------------------------------------------------------
    # 4) 检查 8080
    #    如果还有别的程序占用，只报警，不误杀
    # ------------------------------------------------------------------
    if ss -lntp 2>/dev/null | grep -q ':8080 '
    then
        echo "[WARN] TCP 8080 仍被占用："
        ss -lntp 2>/dev/null \
            | grep ':8080 ' || true
    else
        echo "[PASS] TCP 8080 已释放"
    fi
}

cleanup_left_probe_http
# ===== LEFT_PROBE_HTTP_CLEANUP_END =====


source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

set -eo pipefail

WS="$HOME/ros2_ws"
RUN_DIR="$WS/left_probe_service/run"

stop_pid_file()
{
    local name="$1"
    local file="$RUN_DIR/$name.pid"

    if [[ ! -f "$file" ]]; then
        echo "[skip] $name不是由常驻服务脚本启动"
        return
    fi

    local pid
    pid="$(cat "$file")"

    if kill -0 "$pid" 2>/dev/null; then
        echo "[stop] 停止$name，PID=$pid"

        kill -INT -- "-$pid" 2>/dev/null || true

        for _ in $(seq 1 30); do
            if ! kill -0 "$pid" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM -- "-$pid" 2>/dev/null || true
        fi
    fi

    rm -f "$file"
}

echo "========================================================================"
echo "停止左臂探头常驻服务"
echo "========================================================================"

if ros2 service list 2>/dev/null |
   grep -qx "/left_probe/task/cancel"; then

    timeout 3 ros2 service call \
        /left_probe/task/cancel \
        std_srvs/srv/Trigger \
        "{}" >/dev/null 2>&1 || true

    sleep 0.5
fi

stop_pid_file "http_server"
stop_pid_file "task_server"
stop_pid_file "distance"
stop_pid_file "camera"
stop_pid_file "driver"

echo

# ===== FORCE_STOP_LEFT_D435_BEGIN =====

force_stop_left_d435()
{
    echo
    echo "========================================================================"
    echo "强制停止左侧 D435"
    echo "========================================================================"

    local run_dir="$HOME/ros2_ws/left_probe_service/run"
    local pid
    local cmd

    # ------------------------------------------------------------
    # 1. 如果有 camera.pid，先按PID/进程组关闭
    # ------------------------------------------------------------
    if [[ -f "$run_dir/camera.pid" ]]; then
        pid="$(cat "$run_dir/camera.pid" 2>/dev/null || true)"

        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            cmd="$(
                tr '\0' ' ' \
                < "/proc/$pid/cmdline" \
                2>/dev/null || true
            )"

            echo "[D435 TERM] PID=$pid"
            echo "            $cmd"

            kill -INT -- "-$pid" 2>/dev/null \
                || kill -TERM "$pid" 2>/dev/null \
                || true
        fi
    fi

    sleep 1

    # ------------------------------------------------------------
    # 2. 无论是不是本次start启动，都精确查杀左D435 launch
    #
    # 只匹配：
    # camera_namespace:=left_probe
    # camera_name:=d435
    #
    # 不会匹配右侧 /camera/camera
    # ------------------------------------------------------------
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        [[ "$pid" != "$$" ]] || continue
        [[ -d "/proc/$pid" ]] || continue

        cmd="$(
            tr '\0' ' ' \
            < "/proc/$pid/cmdline" \
            2>/dev/null || true
        )"

        echo "[D435 LAUNCH TERM] PID=$pid"
        echo "                   $cmd"

        kill -INT -- "-$pid" 2>/dev/null \
            || kill -TERM "$pid" 2>/dev/null \
            || true

    done < <(
        pgrep -f \
        'ros2 launch realsense2_camera rs_launch.py.*camera_namespace:=left_probe.*camera_name:=d435' \
        2>/dev/null || true
    )

    sleep 2

    # ------------------------------------------------------------
    # 3. launch如果没带走底层node，再精确杀
    #    __ns:=/left_probe 是左相机唯一标志
    # ------------------------------------------------------------
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        [[ "$pid" != "$$" ]] || continue
        [[ -d "/proc/$pid" ]] || continue

        cmd="$(
            tr '\0' ' ' \
            < "/proc/$pid/cmdline" \
            2>/dev/null || true
        )"

        echo "[D435 NODE TERM] PID=$pid"
        echo "                 $cmd"

        kill -TERM "$pid" 2>/dev/null || true

    done < <(
        pgrep -f \
        'realsense2_camera_node.*__ns:=/left_probe' \
        2>/dev/null || true
    )

    sleep 2

    # ------------------------------------------------------------
    # 4. 仍有残留才KILL
    # ------------------------------------------------------------
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        [[ "$pid" != "$$" ]] || continue

        echo "[D435 LAUNCH KILL] PID=$pid"

        kill -KILL -- "-$pid" 2>/dev/null \
            || kill -KILL "$pid" 2>/dev/null \
            || true

    done < <(
        pgrep -f \
        'ros2 launch realsense2_camera rs_launch.py.*camera_namespace:=left_probe.*camera_name:=d435' \
        2>/dev/null || true
    )

    while read -r pid; do
        [[ -n "$pid" ]] || continue
        [[ "$pid" != "$$" ]] || continue

        echo "[D435 NODE KILL] PID=$pid"

        kill -KILL "$pid" 2>/dev/null || true

    done < <(
        pgrep -f \
        'realsense2_camera_node.*__ns:=/left_probe' \
        2>/dev/null || true
    )

    rm -f "$run_dir/camera.pid"

    sleep 1

    # ------------------------------------------------------------
    # 5. 最终验证
    # ------------------------------------------------------------
    if pgrep -f \
        'ros2 launch realsense2_camera rs_launch.py.*camera_namespace:=left_probe.*camera_name:=d435' \
        >/dev/null 2>&1
    then
        echo "[WARN] 左D435 launch仍有残留"
    elif pgrep -f \
        'realsense2_camera_node.*__ns:=/left_probe' \
        >/dev/null 2>&1
    then
        echo "[WARN] 左D435 node仍有残留"
    else
        echo "[PASS] 左D435进程已完全停止"
    fi
}

force_stop_left_d435

# ===== FORCE_STOP_LEFT_D435_END =====

echo "[DONE] 本脚本启动的常驻进程已停止"
echo "预先存在并被复用的节点不会被关闭"
