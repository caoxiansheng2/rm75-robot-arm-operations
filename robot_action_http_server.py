#!/usr/bin/env python3

import argparse
import json
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger


LEFT_START = "/left_probe/task/start"
LEFT_STATUS = "/left_probe/task/status"
LEFT_CANCEL = "/left_probe/task/cancel"

SUCCESS_STATES = {
    "SUCCESS",
}

FAILED_STATES = {
    "FAILED",
    "REJECTED",
    "CANCELED",
    "CANCELLED",
    "ERROR",
    "G300_ERROR",
    "G300_FAILED",
    "RETURN_FAILED",
    "INCOMPLETE_RETURNED",
}


class RobotActionBridge(Node):

    def __init__(self, workspace: Path):
        super().__init__("robot_action_http_bridge")

        self.ws = workspace
        self.right_script = (
            self.ws / "right_arm_press_button.sh"
        )

        self.turn_knob_script = (
            self.ws / "right_arm_turn_knob.sh"
        )

        self.handoff_file = (
            self.ws
            / "left_probe_service"
            / "run"
            / "pending_http_task.json"
        )

        self.task_log_root = (
            self.ws
            / "left_probe_service"
            / "task_logs"
        )

        self.button_log_root = (
            self.ws
            / "right_qr_roundtrip"
            / "http_tasks"
        )

        self.handoff_file.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.task_log_root.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.button_log_root.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.start_client = self.create_client(
            Trigger,
            LEFT_START,
        )

        self.status_client = self.create_client(
            Trigger,
            LEFT_STATUS,
        )

        self.cancel_client = self.create_client(
            Trigger,
            LEFT_CANCEL,
        )

        self.data_lock = threading.RLock()
        self.record_lock = threading.RLock()

        self.records = {}

        self.distance_m = None
        self.distance_valid = False
        self.distance_stamp = 0.0
        self.pipeline_stamp = 0.0

        self.create_subscription(
            Float32,
            "/left_probe/near_distance_m",
            self.distance_cb,
            10,
        )

        self.create_subscription(
            Bool,
            "/left_probe/distance_valid",
            self.valid_cb,
            10,
        )

        self.get_logger().info(
            "统一机器人动作HTTP桥已启动"
        )

    def distance_cb(self, msg):
        with self.data_lock:
            self.distance_m = float(msg.data)
            self.distance_stamp = time.monotonic()

    def valid_cb(self, msg):
        with self.data_lock:
            self.distance_valid = bool(msg.data)
            self.pipeline_stamp = time.monotonic()

    def health(self):
        with self.data_lock:
            now = time.monotonic()

            pipeline_age = (
                None
                if self.pipeline_stamp <= 0
                else now - self.pipeline_stamp
            )

            distance_age = (
                None
                if self.distance_stamp <= 0
                else now - self.distance_stamp
            )

            pipeline_fresh = (
                pipeline_age is not None
                and pipeline_age <= 1.0
            )

            distance_fresh = (
                distance_age is not None
                and distance_age <= 1.0
            )

            distance_valid = self.distance_valid
            distance_m = self.distance_m

        services = {
            "left_start":
                self.start_client.service_is_ready(),

            "left_status":
                self.status_client.service_is_ready(),

            "left_cancel":
                self.cancel_client.service_is_ready(),

            "right_script":
                self.right_script.is_file(),

            "right_turn_knob_script":
                self.turn_knob_script.is_file(),
        }

        left_ready = (
            services["left_start"]
            and services["left_status"]
            and pipeline_fresh
            and distance_fresh
            and distance_valid
            and distance_m is not None
        )

        return {
            "ok":
                bool(
                    services["left_start"]
                    and services["left_status"]
                    and services["right_script"]
                    and services["right_turn_knob_script"]
                ),

            "left_ready":
                bool(left_ready),

            "right_ready":
                bool(services["right_script"]),

            "task_three_ready":
                bool(
                    services[
                        "right_turn_knob_script"
                    ]
                ),

            "services":
                services,

            "depth_pipeline_fresh":
                bool(pipeline_fresh),

            "depth_pipeline_age_s":
                None
                if pipeline_age is None
                else round(pipeline_age, 3),

            "distance_valid":
                bool(distance_valid),

            "distance_m":
                distance_m,

            "distance_fresh":
                bool(distance_fresh),

            "distance_age_s":
                None
                if distance_age is None
                else round(distance_age, 3),
        }

    @staticmethod
    def parse_message(message):
        if not message:
            return {}

        try:
            value = json.loads(message)

            if isinstance(value, dict):
                return value

            return {
                "value": value
            }

        except Exception:
            return {
                "message": str(message)
            }

    def call_trigger(
        self,
        client,
        timeout=5.0,
    ):
        if not client.service_is_ready():
            if not client.wait_for_service(
                timeout_sec=1.0
            ):
                return False, None, "service_not_ready"

        future = client.call_async(
            Trigger.Request()
        )

        done = threading.Event()

        future.add_done_callback(
            lambda _: done.set()
        )

        if not done.wait(timeout):
            return False, None, "service_timeout"

        try:
            return True, future.result(), ""

        except Exception as exc:
            return (
                False,
                None,
                f"{type(exc).__name__}: {exc}",
            )

    def set_record(
        self,
        task_id,
        **kwargs,
    ):
        with self.record_lock:
            record = self.records.setdefault(
                task_id,
                {
                    "task_id": task_id,
                    "status": "idle",
                    "type": "",
                    "stage": "idle",
                    "message": "",
                },
            )

            record.update(kwargs)
            return dict(record)

    def get_record(self, task_id):
        with self.record_lock:
            record = self.records.get(task_id)

            return (
                None
                if record is None
                else dict(record)
            )

    # ============================================================
    # 好管家 /status 局放结果
    # ============================================================

    @staticmethod
    def _empty_pd_result():
        return {
            "result_available": False,
            "detected": None,
            "discharge_type": None,
            "discharge_type_name": None,
            "basic": None,
            "pulse_count": None,
            "collection_count": None,
        }

    def _latest_g300_terminal_log(
        self,
        task_id,
    ):
        root = (
            self.task_log_root
            / task_id
            / "g300"
        )

        if not root.exists():
            return None

        candidates = list(
            root.glob("*/terminal.log")
        )

        if not candidates:
            return None

        try:
            return max(
                candidates,
                key=lambda path:
                    path.stat().st_mtime,
            )
        except Exception:
            return candidates[-1]

    def completed_g300_results(
        self,
        task_id,
    ):
        """
        从正在写入的G300 terminal.log中提取
        已经完整完成的检测通道。

        未完成的通道绝不返回。
        """

        log_file = (
            self._latest_g300_terminal_log(
                task_id
            )
        )

        if log_file is None:
            return {}

        try:
            text = log_file.read_text(
                encoding="utf-8",
                errors="replace",
            )
        except Exception:
            return {}

        channel_headers = (
            (
                "开放式超声波 完整结果",
                "open_ultrasonic_result",
            ),
            (
                "地电波 TEV 完整结果",
                "tev_result",
            ),
            (
                "接触式超声波 完整结果",
                "contact_ultrasonic_result",
            ),
            (
                "特高频 UHF 完整结果",
                "uhf_result",
            ),
        )

        results = {}

        current_field = None
        current = {}

        def commit():
            nonlocal current_field
            nonlocal current

            if current_field is None:
                return

            # 只有“完整结果”的关键字段全部拿到，
            # 才认为该通道真正完成。
            if not all(
                key in current
                for key in (
                    "discharge_type",
                    "discharge_type_name",
                    "basic",
                    "pulse_count",
                )
            ):
                return

            dtype = int(
                current["discharge_type"]
            )

            # 9 = 分析未完成，绝不作为有效结果。
            if dtype == 9:
                return

            results[current_field] = {
                "result_available": True,
                "detected":
                    dtype in (1, 2, 3, 4),

                "discharge_type":
                    dtype,

                "discharge_type_name":
                    str(
                        current[
                            "discharge_type_name"
                        ]
                    ),

                "basic":
                    current["basic"],

                "pulse_count":
                    int(
                        current[
                            "pulse_count"
                        ]
                    ),

                # 能进入“完整结果”块，
                # 说明该通道已经完成20周期。
                "collection_count":
                    20,
            }

            current_field = None
            current = {}

        for raw_line in text.splitlines():
            line = raw_line.strip()

            new_field = None

            for marker, field in (
                channel_headers
            ):
                if marker in line:
                    new_field = field
                    break

            if new_field is not None:
                commit()

                current_field = (
                    new_field
                )
                current = {}
                continue

            if current_field is None:
                continue

            match = re.search(
                r"最终类型\s*:\s*"
                r"([0-9]+)\s*"
                r"\(([^)]+)\)",
                line,
            )

            if match:
                current[
                    "discharge_type"
                ] = int(
                    match.group(1)
                )

                current[
                    "discharge_type_name"
                ] = (
                    match.group(2).strip()
                )

                continue

            match = re.search(
                r"基本数据\s*:\s*"
                r"([-+0-9.eE]+)",
                line,
            )

            if match:
                try:
                    current["basic"] = (
                        float(
                            match.group(1)
                        )
                    )
                except Exception:
                    pass

                continue

            match = re.search(
                r"周期内脉冲数\s*:\s*"
                r"([0-9]+)",
                line,
            )

            if match:
                current[
                    "pulse_count"
                ] = int(
                    match.group(1)
                )

                # 到这里完整结果关键字段已经齐全。
                commit()

        commit()

        return results

    def build_public_status(
        self,
        task_id,
    ):
        """
        好管家固定报文：

        task_id
        status
        type
        message

        detection任务根据完成情况追加结果字段。
        """

        record = self.get_record(
            task_id
        )

        if record is None:
            return {
                "task_id": task_id,
                "status": "failed",
                "type": "detection",
                "message": "Task not found.",
            }

        raw_status = str(
            record.get(
                "status",
                "executing",
            )
        ).strip().lower()

        # 对外只允许这三个状态。
        if raw_status == "success":
            status = "success"
        elif raw_status == "failed":
            status = "failed"
        else:
            status = "executing"

        task_type = str(
            record.get(
                "type",
                "detection",
            )
        )

        if status == "success":
            message = (
                "Task completed successfully."
            )

        elif status == "failed":
            message = str(
                record.get(
                    "message",
                    "",
                )
                or
                "Task completed with "
                "status failed."
            )

        else:
            message = (
                "Task is executing."
            )

        payload = {
            "task_id": task_id,
            "status": status,
            "type": task_type,
            "message": message,
        }

        # button没有局放结果字段。
        if task_type != "detection":
            return payload

        completed = (
            self.completed_g300_results(
                task_id
            )
        )

        ordered_fields = (
            "open_ultrasonic_result",
            "tev_result",
            "contact_ultrasonic_result",
            "uhf_result",
        )

        if status in (
            "executing",
            "failed",
        ):
            # 只返回已经完整完成的通道。
            for field in ordered_fields:
                if field in completed:
                    payload[field] = (
                        completed[field]
                    )

            return payload

        # SUCCESS：
        # 正式结构始终返回全部四个字段。
        for field in ordered_fields:
            payload[field] = (
                completed.get(
                    field,
                    self._empty_pd_result(),
                )
            )

        return payload


    def write_left_handoff(
        self,
        task_id,
        detection_type,
    ):
        payload = {
            "task_id": task_id,
            "detection_type": detection_type,
        }

        tmp = self.handoff_file.with_name(
            self.handoff_file.name + ".tmp"
        )

        tmp.write_text(
            json.dumps(
                payload,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )

        tmp.replace(
            self.handoff_file
        )

    def remove_handoff(self):
        try:
            self.handoff_file.unlink()
        except FileNotFoundError:
            pass

    def load_left_result(self, task_id):
        result_file = (
            self.task_log_root
            / task_id
            / "task_result.json"
        )

        if not result_file.exists():
            return None

        try:
            return json.loads(
                result_file.read_text(
                    encoding="utf-8"
                )
            )
        except Exception:
            return None

    # ============================================================
    # 左臂 detection
    # ============================================================

    def consume_next_detection_type(
        self,
        default="all",
    ):
        """
        读取一次性演示模式。

        文件不存在：
            返回 all

        文件存在且为合法单通道：
            返回对应通道
            并立即删除文件

        只应在真正准备启动 detection 时调用。
        """

        mode_file = (
            self.ws
            / "left_probe_service"
            / "run"
            / "next_detection_type"
        )

        if not mode_file.exists():
            return default

        try:
            mode = mode_file.read_text(
                encoding="utf-8"
            ).strip()

        except Exception as exc:
            self.get_logger().error(
                "[DEMO MODE] 读取失败："
                f"{exc}"
            )
            return default

        valid = {
            "open_ultrasonic",
            "tev",
            "contact_ultrasonic",
            "uhf",
        }

        try:
            mode_file.unlink()
        except Exception as exc:
            self.get_logger().warning(
                "[DEMO MODE] 删除一次性标志失败："
                f"{exc}"
            )

        if mode not in valid:
            self.get_logger().error(
                "[DEMO MODE] 非法模式："
                f"{mode}，本次恢复all"
            )
            return default

        self.get_logger().warning(
            "[DEMO MODE] "
            "下一次演示模式已消费："
            f"{mode}"
        )

        return mode


    def execute_detection(
        self,
        task_id,
        detection_type="all",
        timeout=360.0,
    ):
        self.set_record(
            task_id,
            status="executing",
            type="detection",
            detection_type=detection_type,
            stage="starting",
            message="left detection starting",
        )

        try:
            self.write_left_handoff(
                task_id,
                detection_type,
            )
        except Exception as exc:
            return self.finish_failed(
                task_id,
                "detection",
                f"handoff failed: {exc}",
            )

        ok, response, error = self.call_trigger(
            self.start_client,
            timeout=5.0,
        )

        if not ok:
            self.remove_handoff()

            return self.finish_failed(
                task_id,
                "detection",
                f"left start service failed: {error}",
            )

        if not response.success:
            self.remove_handoff()

            detail = self.parse_message(
                response.message
            )

            return self.finish_failed(
                task_id,
                "detection",
                detail.get(
                    "message",
                    response.message or "task rejected",
                ),
            )

        deadline = (
            time.monotonic()
            + timeout
        )

        last_state = ""

        while time.monotonic() < deadline:
            ok, response, error = self.call_trigger(
                self.status_client,
                timeout=3.0,
            )

            if not ok:
                time.sleep(0.25)
                continue

            payload = self.parse_message(
                response.message
            )

            state = str(
                payload.get(
                    "state",
                    payload.get(
                        "status",
                        "",
                    ),
                )
            ).strip().upper()

            if state and state != last_state:
                last_state = state

                self.get_logger().info(
                    f"[LEFT {task_id}] {state}"
                )

                self.set_record(
                    task_id,
                    status="executing",
                    type="detection",
                    detection_type=detection_type,
                    stage=state.lower(),
                    message=payload.get(
                        "message",
                        "",
                    ),
                )

            if state in SUCCESS_STATES:
                result = self.load_left_result(
                    task_id
                )

                self.set_record(
                    task_id,
                    status="success",
                    type="detection",
                    detection_type=detection_type,
                    stage="success",
                    message="left detection completed",
                    result=result,
                )

                return True

            if (
                state in FAILED_STATES
                or state.startswith("FAILED")
                or state.startswith("REJECTED")
                or state.startswith("ERROR")
            ):
                result = self.load_left_result(
                    task_id
                )

                self.set_record(
                    task_id,
                    status="failed",
                    type="detection",
                    detection_type=detection_type,
                    stage=state.lower(),
                    message=payload.get(
                        "message",
                        state,
                    ),
                    result=result,
                )

                return False

            time.sleep(0.25)

        return self.finish_failed(
            task_id,
            "detection",
            "left detection timeout",
        )

    # ============================================================
    # 右臂 button
    # ============================================================

    def execute_button(
        self,
        task_id,
    ):
        self.set_record(
            task_id,
            status="executing",
            type="button",
            stage="starting",
            message="right arm task starting",
        )

        if not self.right_script.is_file():
            return self.finish_failed(
                task_id,
                "button",
                f"missing {self.right_script}",
            )

        task_dir = (
            self.button_log_root
            / task_id
        )

        task_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        log_file = (
            task_dir
            / "right_arm_press_button.log"
        )

        self.set_record(
            task_id,
            status="executing",
            type="button",
            stage="right_arm_running",
            message="right arm QR/button task executing",
            log=str(log_file),
        )

        self.get_logger().info(
            f"[RIGHT START] task_id={task_id}"
        )

        try:
            with log_file.open(
                "w",
                encoding="utf-8",
                buffering=1,
            ) as log:

                proc = subprocess.run(
                    [
                        "bash",
                        str(self.right_script),
                    ],
                    cwd=str(self.ws),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=480,
                    check=False,
                )

        except subprocess.TimeoutExpired:
            return self.finish_failed(
                task_id,
                "button",
                "right arm task timeout",
            )

        except Exception as exc:
            return self.finish_failed(
                task_id,
                "button",
                (
                    f"right arm exception: "
                    f"{type(exc).__name__}: {exc}"
                ),
            )

        if proc.returncode != 0:
            return self.finish_failed(
                task_id,
                "button",
                (
                    "right arm task failed, "
                    f"returncode={proc.returncode}"
                ),
            )

        self.set_record(
            task_id,
            status="success",
            type="button",
            stage="success",
            message="right arm task completed",
            log=str(log_file),
        )

        self.get_logger().info(
            f"[RIGHT SUCCESS] task_id={task_id}"
        )

        return True

    # ============================================================
    # 右臂 task_three：旋钮任务
    # ============================================================

    def execute_task_three(
        self,
        task_id,
    ):
        self.set_record(
            task_id,
            status="executing",
            type="task_three",
            stage="preparing_right_arm",
            message="Right arm knob task is preparing.",
        )

        if not self.turn_knob_script.is_file():
            return self.finish_failed(
                task_id,
                "task_three",
                f"missing {self.turn_knob_script}",
            )

        task_dir = (
            self.ws
            / "right_turn_knob"
            / "http_tasks"
            / task_id
        )

        task_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        cleanup_log = (
            task_dir
            / "cleanup_press_button_stack.log"
        )

        log_file = (
            task_dir
            / "right_arm_turn_knob.log"
        )

        # --------------------------------------------------------
        # right_arm_press_button.sh执行后会保留headless组件。
        #
        # turn_knob.sh使用的是专用turn_knob.launch.py，
        # 因此开始旋钮任务前先清理button任务留下的右臂栈，
        # 避免两个rm_driver/camera/TF链同时存在。
        # --------------------------------------------------------

        if self.right_script.is_file():

            self.get_logger().info(
                "[TASK_THREE] "
                "清理right_arm_press_button遗留组件"
            )

            try:
                with cleanup_log.open(
                    "w",
                    encoding="utf-8",
                    buffering=1,
                ) as log:

                    cleanup = subprocess.run(
                        [
                            "bash",
                            str(self.right_script),
                            "stop",
                        ],
                        cwd=str(self.ws),
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        timeout=90,
                        check=False,
                    )

                self.get_logger().info(
                    "[TASK_THREE] "
                    "button stack cleanup "
                    f"returncode={cleanup.returncode}"
                )

            except Exception as exc:
                self.get_logger().warning(
                    "[TASK_THREE] "
                    "button stack cleanup异常，"
                    "继续由旋钮脚本自身检查："
                    f"{type(exc).__name__}: {exc}"
                )

        self.set_record(
            task_id,
            status="executing",
            type="task_three",
            stage="right_turn_knob_running",
            message="Right arm knob task is executing.",
            log=str(log_file),
        )

        self.get_logger().info(
            "[TASK_THREE START] "
            f"task_id={task_id}"
        )

        # --------------------------------------------------------
        # 真正执行用户现有的旋钮脚本。
        #
        # 脚本自身：
        # - flock防右臂动作冲突
        # - 启动turn_knob.launch.py
        # - 等待/rm_driver和camera
        # - 等待/rm75_turn_knob/execute
        # - 等待knob_tag
        # - 执行旋钮
        # - 返回固定姿态
        # - EXIT trap清理专用launch
        # --------------------------------------------------------

        try:
            with log_file.open(
                "w",
                encoding="utf-8",
                buffering=1,
            ) as log:

                proc = subprocess.run(
                    [
                        "bash",
                        str(
                            self.turn_knob_script
                        ),
                    ],
                    cwd=str(self.ws),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=480,
                    check=False,
                )

        except subprocess.TimeoutExpired:
            return self.finish_failed(
                task_id,
                "task_three",
                "Right arm knob task timeout.",
            )

        except Exception as exc:
            return self.finish_failed(
                task_id,
                "task_three",
                (
                    "Right arm knob exception: "
                    f"{type(exc).__name__}: "
                    f"{exc}"
                ),
            )

        if proc.returncode != 0:
            return self.finish_failed(
                task_id,
                "task_three",
                (
                    "Right arm knob task failed, "
                    f"returncode="
                    f"{proc.returncode}"
                ),
            )

        self.set_record(
            task_id,
            status="success",
            type="task_three",
            stage="success",
            message=(
                "Right arm knob task "
                "completed successfully."
            ),
            log=str(log_file),
        )

        self.get_logger().info(
            "[TASK_THREE SUCCESS] "
            f"task_id={task_id}"
        )

        return True


    def finish_failed(
        self,
        task_id,
        task_type,
        message,
    ):
        self.set_record(
            task_id,
            status="failed",
            type=task_type,
            stage="failed",
            message=message,
        )

        self.get_logger().error(
            f"[{task_type.upper()} FAILED] "
            f"task_id={task_id}: {message}"
        )

        return False


class RobotHttpServer(
    ThreadingHTTPServer
):
    allow_reuse_address = True
    daemon_threads = True


def make_handler(bridge):

    # 左右动作当前按业务串行执行。
    execution_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):

        server_version = (
            "RobotActionHTTP/1.0"
        )

        def log_message(
            self,
            fmt,
            *args,
        ):
            bridge.get_logger().info(
                "HTTP "
                + self.address_string()
                + " "
                + (fmt % args)
            )

        def send_json(
            self,
            code,
            payload,
        ):
            body = json.dumps(
                payload,
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")

            try:
                self.send_response(code)

                self.send_header(
                    "Content-Type",
                    (
                        "application/json; "
                        "charset=utf-8"
                    ),
                )

                self.send_header(
                    "Content-Length",
                    str(len(body)),
                )

                self.send_header(
                    "Connection",
                    "close",
                )

                self.end_headers()

                self.wfile.write(body)

            except (
                BrokenPipeError,
                ConnectionResetError,
            ):
                bridge.get_logger().warning(
                    "好管家HTTP连接已提前断开"
                )

        def read_json(self):
            length = int(
                self.headers.get(
                    "Content-Length",
                    "0",
                )
                or 0
            )

            if length <= 0:
                return {}

            raw = self.rfile.read(length)

            return json.loads(
                raw.decode("utf-8")
            )

        def do_POST(self):
            path = urlparse(
                self.path
            ).path.rstrip("/")

            if path != "/start_task":
                self.send_json(
                    404,
                    {
                        "status": "failed",
                        "message": "not found",
                    },
                )
                return

            try:
                payload = self.read_json()
            except Exception as exc:
                self.send_json(
                    400,
                    {
                        "status": "failed",
                        "message":
                            f"invalid json: {exc}",
                    },
                )
                return

            task_type = str(
                payload.get(
                    "task_type",
                    "",
                )
            ).strip()

            task_id = str(
                payload.get(
                    "task_id",
                    "",
                )
            ).strip()

            if task_type not in (
                "detection",
                "button",
                "task_three",
            ):
                self.send_json(
                    400,
                    {
                        "task_id": task_id,
                        "status": "failed",
                        "message":
                            "task_type must be detection, button or task_three",
                    },
                )
                return

            if not task_id:
                self.send_json(
                    400,
                    {
                        "status": "failed",
                        "message":
                            "task_id is required",
                    },
                )
                return

            old = bridge.get_record(
                task_id
            )

            if old is not None:
                code = (
                    200
                    if old.get("status")
                    == "success"
                    else 500
                    if old.get("status")
                    == "failed"
                    else 409
                )

                self.send_json(
                    code,
                    bridge.build_public_status(
                        task_id
                    ),
                )
                return

            if not execution_lock.acquire(
                blocking=False
            ):
                self.send_json(
                    409,
                    {
                        "task_id": task_id,
                        "status": "failed",
                        "type": task_type,
                        "message":
                            "another robot task is executing",
                    },
                )
                return

            try:
                if task_type == "detection":


                    # 正常好管家 detection 默认执行四通道。

                    # 如果本地演示脚本设置了一次性模式，

                    # 本次只执行指定单通道，随后自动恢复all。

                    detection_type = (

                        bridge.consume_next_detection_type(

                            default="all"

                        )

                    )


                    success = (

                        bridge.execute_detection(

                            task_id,

                            detection_type,

                        )

                    )


                elif task_type == "button":
                    success = (
                        bridge.execute_button(
                            task_id
                        )
                    )

                else:
                    # task_type == "task_three"
                    success = (
                        bridge.execute_task_three(
                            task_id
                        )
                    )

            finally:
                execution_lock.release()

            result = (
                bridge.build_public_status(
                    task_id
                )
            )

            self.send_json(
                200 if success else 500,
                result,
            )

        def do_GET(self):
            parsed = urlparse(
                self.path
            )

            path = parsed.path.rstrip("/")

            if path == "/health":
                self.send_json(
                    200,
                    bridge.health(),
                )
                return

            if path == "/status":
                query = parse_qs(
                    parsed.query
                )

                task_id = (
                    query.get(
                        "task_id",
                        [""],
                    )[0]
                )

                if not task_id:
                    self.send_json(
                        400,
                        {
                            "status": "failed",
                            "message":
                                "task_id required",
                        },
                    )
                    return

                payload = (
                    bridge.build_public_status(
                        task_id
                    )
                )

                self.send_json(
                    200,
                    payload,
                )
                return

            self.send_json(
                404,
                {
                    "status": "failed",
                    "message": "not found",
                },
            )

    return Handler


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--host",
        default="127.0.0.1",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=8080,
    )

    parser.add_argument(
        "--workspace",
        default=str(
            Path.home()
            / "ros2_ws"
        ),
    )

    args = parser.parse_args()

    workspace = Path(
        args.workspace
    ).expanduser()

    rclpy.init()

    bridge = RobotActionBridge(
        workspace
    )

    # 先bind端口，再启ROS线程，
    # 避免8080占用时出现executor abort。
    try:
        server = RobotHttpServer(
            (
                args.host,
                args.port,
            ),
            make_handler(bridge),
        )

    except Exception:
        bridge.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        raise

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(
        bridge
    )

    ros_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )

    ros_thread.start()

    bridge.get_logger().info(
        "============================================"
    )
    bridge.get_logger().info(
        "统一左右臂好管家接口启动"
    )
    bridge.get_logger().info(
        f"http://{args.host}:{args.port}"
    )
    bridge.get_logger().info(
        "detection -> 左臂局放"
    )
    bridge.get_logger().info(
        "button    -> 右臂QR/按钮"
    )
    bridge.get_logger().info(
        "============================================"
    )

    try:
        server.serve_forever(
            poll_interval=0.2
        )

    except KeyboardInterrupt:
        pass

    finally:
        server.shutdown()
        server.server_close()

        executor.shutdown()
        bridge.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
