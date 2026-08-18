#!/usr/bin/env bash
set -Eeo pipefail

WS="${HOME}/ros2_ws"
LOG_DIR="${WS}/right_press_button/$(date +%Y%m%d_%H%M%S)"
PID_FILE="${WS}/right_press_button.launch.pid"
TIMEOUT_SECONDS=300

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/vision_bringup.log"
LAUNCH_PID=""

stop_launch()
{
    local pid="${LAUNCH_PID}"
    if [[ -z "${pid}" && -f "${PID_FILE}" ]]; then
        pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
    fi
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        echo "[STOP] rm75_vision_bringup PID=${pid}"
        kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
        sleep 2
    fi
    rm -f "${PID_FILE}"
}

if [[ "${1:-}" == "stop" ]]; then
    stop_launch
    exit 0
fi

exec 9>/tmp/right_arm_motion.lock
flock -n 9 || { echo "[FAIL] Another right-arm task is running."; exit 1; }
trap stop_launch EXIT
trap 'exit 130' INT TERM

echo "[START] Button press and automatic joint return"
echo "[INFO] Log: ${LOG_FILE}"
setsid ros2 launch rm75_fixed_grasp rm75_vision_bringup.launch.py \
    start_auto_grasp:=true auto_execute:=true \
    allow_motion:=true dry_run:=false execute_contact:=true \
    >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!
echo "${LAUNCH_PID}" >"${PID_FILE}"

for _ in $(seq 1 $((TIMEOUT_SECONDS * 2))); do
    if grep -q 'Automatic motion completed:' "${LOG_FILE}" 2>/dev/null; then
        echo "[PASS] Button pressed and robot returned to configured joints."
        exit 0
    fi
    if grep -Eq \
        'Automatic motion failed|Execution failed|Automatic execute service call failed' \
        "${LOG_FILE}" 2>/dev/null
    then
        echo "[FAIL] Motion node reported an error:"
        tail -n 80 "${LOG_FILE}"
        exit 1
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "[FAIL] Bringup exited unexpectedly:"
        tail -n 80 "${LOG_FILE}"
        exit 1
    fi
    sleep 0.5
done

echo "[FAIL] Task timed out after ${TIMEOUT_SECONDS}s."
tail -n 80 "${LOG_FILE}"
exit 1
