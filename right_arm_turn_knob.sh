#!/usr/bin/env bash
set -Eeo pipefail

WS="${HOME}/ros2_ws"
LOG_DIR="${WS}/right_turn_knob/$(date +%Y%m%d_%H%M%S)"
PID_FILE="/tmp/right_arm_turn_knob.pid"

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
mkdir -p "${LOG_DIR}"
LAUNCH_PID=""

cleanup()
{
    if [[ -n "${LAUNCH_PID}" ]]; then
        kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || \
            kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    fi
    if [[ -f "${PID_FILE}" ]] && [[ "$(cat "${PID_FILE}" 2>/dev/null)" == "$$" ]]; then
        rm -f "${PID_FILE}"
    fi
}

stop_task()
{
    if [[ ! -f "${PID_FILE}" ]]; then
        echo "[INFO] No recorded rotary-knob task PID."
        return 1
    fi
    local task_pid
    task_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
    if [[ -z "${task_pid}" ]] || ! kill -0 "${task_pid}" 2>/dev/null; then
        echo "[INFO] Recorded rotary-knob task is no longer running."
        rm -f "${PID_FILE}"
        return 0
    fi
    echo "[STOP] Sending SIGINT to rotary-knob task PID=${task_pid}"
    kill -INT "${task_pid}"
    return 0
}

if [[ "${1:-}" == "stop" ]]; then
    stop_task
    exit $?
fi

exec 9>/tmp/right_arm_motion.lock
flock -n 9 || { echo "[FAIL] Another right-arm motion task is running."; exit 1; }
echo "$$" >"${PID_FILE}"
trap cleanup EXIT
trap 'exit 130' INT TERM

echo "========================================================================"
echo "RM75 rotary-knob task using AprilTag ID 01"
echo "Logs: ${LOG_DIR}"
echo "========================================================================"

# The dedicated launch starts exactly one camera, the ID-1 detector, hand-eye
# TF, RM75 base stack and the independent turn_knob_node.
setsid ros2 launch rm75_fixed_grasp turn_knob.launch.py \
    allow_motion:=true dry_run:=false \
    >"${LOG_DIR}/bringup.log" 2>&1 &
LAUNCH_PID=$!

echo "[WAIT] Waiting for RM driver and color camera."
BASE_READY=false
for _ in $(seq 1 180); do
    NODES="$(ros2 node list 2>/dev/null || true)"
    if grep -q '^/rm_driver$' <<<"${NODES}" && \
       grep -q '^/camera/camera$' <<<"${NODES}"; then
        BASE_READY=true
        break
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "[FAIL] Bringup exited during startup."
        tail -n 100 "${LOG_DIR}/bringup.log"
        exit 1
    fi
    sleep 0.5
done
${BASE_READY} || { echo "[FAIL] Driver/camera startup timed out."; exit 1; }

echo "[WAIT] Waiting for /rm75_turn_knob/execute and knob_tag."
TARGET_READY=false
for _ in $(seq 1 240); do
    SERVICE_OK=false
    TAG_OK=false
    SERVICE_OUTPUT="$(ros2 service list 2>/dev/null || true)"
    grep -q '^/rm75_turn_knob/execute$' <<<"${SERVICE_OUTPUT}" && SERVICE_OK=true
    # Do not use `tf2_echo | grep -q` here. With pipefail enabled, grep exits
    # after the first match and tf2_echo can receive SIGPIPE, making a valid TF
    # look like a failed pipeline. RK startup can also take more than 2 s.
    TF_OUTPUT="$(timeout 5 ros2 run tf2_ros tf2_echo \
        base_link knob_tag 2>/dev/null || true)"
    grep -q 'Translation:' <<<"${TF_OUTPUT}" && TAG_OK=true
    if ${SERVICE_OK} && ${TAG_OK}; then
        TARGET_READY=true
        break
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "[FAIL] Dedicated knob launch exited."
        tail -n 150 "${LOG_DIR}/bringup.log"
        exit 1
    fi
    sleep 0.5
done
${TARGET_READY} || { echo "[FAIL] AprilTag ID 01 was not detected."; exit 1; }

echo "[RUN] Executing rotary-knob sequence."
RESULT="$(timeout 360 ros2 service call \
    /rm75_turn_knob/execute std_srvs/srv/Trigger '{}')"
echo "${RESULT}"
if ! grep -q 'success=True' <<<"${RESULT}"; then
    echo "[FAIL] Rotary-knob task failed."
    tail -n 150 "${LOG_DIR}/bringup.log"
    exit 1
fi

echo "[PASS] Knob operation completed and arm returned to fixed pose."
