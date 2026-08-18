#!/usr/bin/env python3

import argparse
import json
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


START_SERVICE = "/left_probe/task/start"
STATUS_SERVICE = "/left_probe/task/status"
CANCEL_SERVICE = "/left_probe/task/cancel"

VALID_DETECTION_TYPES = {
    "open_ultrasonic",
    "tev",
    "contact_ultrasonic",
    "uhf",
    "all",
}

SUCCESS_STATES = {
    "SUCCESS",
}

FAILED_STATES = {
    "FAILED",
    "REJECTED",
    "CANCELED",
    "CANCELLED",
    "ERROR",
    "TIMEOUT",
    "INCOMPLETE_RETURNED",
    "RETURN_FAILED",
    "G300_FAILED",
}


def normalize_state(value):
    if value is None:
        return ""
    return str(value).strip().upper()


def is_success_state(state):
    return normalize_state(state) in SUCCESS_STATES


def is_failed_state(state):
    state = normalize_state(state)

    if state in FAILED_STATES:
        return True

    return (
        state.startswith("FAILED")
        or state.startswith("REJECTED")
        or state.startswith("CANCEL")
        or state.startswith("ERROR")
    )


class LeftProbeHttpBridge(Node):

    def __init__(
        self,
        workspace,
        task_timeout,
        poll_interval,
    ):
        super().__init__(
            "left_probe_http_bridge"
        )

        self.workspace = Path(
            workspace
        ).expanduser()

        self.task_timeout = float(
            task_timeout
        )

        self.poll_interval = float(
            poll_interval
        )

        self.run_dir = (
            self.workspace
            / "left_probe_service"
            / "run"
        )

        self.task_log_dir = (
            self.workspace
            / "left_probe_service"
            / "task_logs"
        )

        self.run_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.task_log_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.handoff_file = (
            self.run_dir
            / "pending_http_task.json"
        )

        self.start_client = self.create_client(
            Trigger,
            START_SERVICE,
        )

        self.status_client = self.create_client(
            Trigger,
            STATUS_SERVICE,
        )

        self.cancel_client = self.create_client(
            Trigger,
            CANCEL_SERVICE,
        )

        self.data_lock = threading.RLock()
        self.records_lock = threading.RLock()

        self.distance_m = None
        self.distance_valid = False
        self.distance_stamp = 0.0
        self.pipeline_stamp = 0.0

        self.records = {}

        self.create_subscription(
            Float32,
            "/left_probe/near_distance_m",
            self.distance_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/left_probe/distance_valid",
            self.distance_valid_callback,
            10,
        )

    # ============================================================
    # D435 health
    # ============================================================

    def distance_callback(self, msg):
        with self.data_lock:
            self.distance_m = float(
                msg.data
            )
            self.distance_stamp = (
                time.monotonic()
            )

    def distance_valid_callback(self, msg):
        with self.data_lock:
            self.distance_valid = bool(
                msg.data
            )
            self.pipeline_stamp = (
                time.monotonic()
            )

    def health(self):
        with self.data_lock:
            now = time.monotonic()

            if self.pipeline_stamp > 0:
                pipeline_age = (
                    now
                    - self.pipeline_stamp
                )
            else:
                pipeline_age = None

            if self.distance_stamp > 0:
                distance_age = (
                    now
                    - self.distance_stamp
                )
            else:
                distance_age = None

            pipeline_fresh = (
                pipeline_age is not None
                and pipeline_age <= 1.0
            )

            distance_fresh = (
                distance_age is not None
                and distance_age <= 1.0
            )

            distance_valid = (
                self.distance_valid
            )

            distance_m = (
                self.distance_m
            )

        services = {
            "start":
                self.start_client.service_is_ready(),

            "status":
                self.status_client.service_is_ready(),

            "cancel":
                self.cancel_client.service_is_ready(),
        }

        ok = (
            services["start"]
            and services["status"]
            and pipeline_fresh
        )

        ready = (
            ok
            and distance_valid
            and distance_fresh
            and distance_m is not None
        )

        return {
            "ok":
                bool(ok),

            "ready":
                bool(ready),

            "services":
                services,

            "depth_pipeline_fresh":
                bool(pipeline_fresh),

            "depth_pipeline_age_s":
                (
                    None
                    if pipeline_age is None
                    else round(
                        pipeline_age,
                        3,
                    )
                ),

            "distance_valid":
                bool(distance_valid),

            "distance_m":
                distance_m,

            "distance_fresh":
                bool(distance_fresh),

            "distance_age_s":
                (
                    None
                    if distance_age is None
                    else round(
                        distance_age,
                        3,
                    )
                ),
        }

    # ============================================================
    # Record
    # ============================================================

    def set_record(
        self,
        task_id,
        **kwargs,
    ):
        with self.records_lock:
            record = self.records.setdefault(
                task_id,
                {
                    "task_id":
                        task_id,

                    "status":
                        "idle",

                    "type":
                        "detection",

                    "detection_type":
                        "all",

                    "stage":
                        "idle",

                    "message":
                        "",
                },
            )

            record.update(
                kwargs
            )

            return dict(
                record
            )

    def get_record(
        self,
        task_id,
    ):
        with self.records_lock:
            value = self.records.get(
                task_id
            )

            if value is None:
                return None

            return dict(
                value
            )

    # ============================================================
    # Trigger
    # ============================================================

    @staticmethod
    def parse_message(message):
        if not message:
            return {}

        try:
            value = json.loads(
                message
            )

            if isinstance(
                value,
                dict,
            ):
                return value

            return {
                "value": value
            }

        except Exception:
            return {
                "message":
                    str(message)
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
                return (
                    False,
                    None,
                    "service_not_ready",
                )

        future = client.call_async(
            Trigger.Request()
        )

        done = threading.Event()

        future.add_done_callback(
            lambda _: done.set()
        )

        if not done.wait(
            timeout
        ):
            return (
                False,
                None,
                "service_timeout",
            )

        try:
            response = future.result()

        except Exception as exc:
            return (
                False,
                None,
                (
                    f"{type(exc).__name__}: "
                    f"{exc}"
                ),
            )

        return (
            True,
            response,
            "",
        )

    # ============================================================
    # Handoff
    # ============================================================

    def write_handoff(
        self,
        task_id,
        detection_type,
    ):
        payload = {
            "task_id":
                task_id,

            "detection_type":
                detection_type,
        }

        tmp = (
            self.handoff_file
            .with_name(
                self.handoff_file.name
                + ".tmp"
            )
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

    # ============================================================
    # Task result
    # ============================================================

    def load_task_result(
        self,
        task_id,
    ):
        result_file = (
            self.task_log_dir
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
    # 同步执行
    # ============================================================

    def execute_detection_sync(
        self,
        task_id,
        detection_type,
    ):
        self.set_record(
            task_id,
            status="executing",
            type="detection",
            detection_type=detection_type,
            stage="starting",
            message="Task starting",
        )

        self.get_logger().info(
            "================================================"
        )

        self.get_logger().info(
            "收到同步局放任务："
            f"task_id={task_id}, "
            f"detection_type={detection_type}"
        )

        # --------------------------------------------------------
        # 参数交接
        # --------------------------------------------------------

        try:
            self.write_handoff(
                task_id,
                detection_type,
            )

        except Exception as exc:
            message = (
                "write_handoff_failed: "
                f"{exc}"
            )

            self.set_record(
                task_id,
                status="failed",
                stage="handoff_failed",
                message=message,
            )

            return {
                "success": False,
                "http_code": 500,
                "message": message,
            }

        # --------------------------------------------------------
        # Start
        # --------------------------------------------------------

        ok, response, error = (
            self.call_trigger(
                self.start_client,
                timeout=5.0,
            )
        )

        if not ok:
            self.remove_handoff()

            message = (
                "start_service_failed: "
                f"{error}"
            )

            self.set_record(
                task_id,
                status="failed",
                stage="start_failed",
                message=message,
            )

            return {
                "success": False,
                "http_code": 503,
                "message": message,
            }

        start_payload = (
            self.parse_message(
                response.message
            )
        )

        if not response.success:
            self.remove_handoff()

            message = (
                start_payload.get(
                    "message"
                )
                or response.message
                or "task_rejected"
            )

            self.set_record(
                task_id,
                status="failed",
                stage="rejected",
                message=message,
            )

            self.get_logger().error(
                "Task Server拒绝任务："
                f"{message}"
            )

            return {
                "success": False,
                "http_code": 409,
                "message": message,
            }

        self.set_record(
            task_id,
            status="executing",
            stage="queued",
            message="Task accepted",
        )

        self.get_logger().info(
            "Task Server已接受任务，"
            "开始同步等待完整执行结束"
        )

        deadline = (
            time.monotonic()
            + self.task_timeout
        )

        last_state = ""
        consecutive_status_errors = 0

        while (
            time.monotonic()
            < deadline
        ):
            ok, response, error = (
                self.call_trigger(
                    self.status_client,
                    timeout=3.0,
                )
            )

            if not ok:
                consecutive_status_errors += 1

                if (
                    consecutive_status_errors == 1
                    or
                    consecutive_status_errors % 10
                    == 0
                ):
                    self.get_logger().warning(
                        "状态查询失败："
                        f"{error}"
                    )

                time.sleep(
                    self.poll_interval
                )
                continue

            consecutive_status_errors = 0

            payload = (
                self.parse_message(
                    response.message
                )
            )

            state = normalize_state(
                payload.get(
                    "state",
                    payload.get(
                        "status",
                        "",
                    ),
                )
            )

            if (
                state
                and
                state != last_state
            ):
                last_state = state

                self.get_logger().info(
                    f"[TASK STATE] {state}"
                )

                self.set_record(
                    task_id,
                    status="executing",
                    stage=state.lower(),
                    message=payload.get(
                        "message",
                        "",
                    ),
                )

            # ----------------------------------------------------
            # 成功
            # ----------------------------------------------------

            if is_success_state(
                state
            ):
                task_result = (
                    self.load_task_result(
                        task_id
                    )
                )

                self.set_record(
                    task_id,
                    status="success",
                    stage="success",
                    message=(
                        "Task completed successfully"
                    ),
                    result=task_result,
                )

                self.get_logger().info(
                    "同步局放任务成功："
                    f"{task_id}"
                )

                self.get_logger().info(
                    "================================================"
                )

                return {
                    "success": True,
                    "http_code": 200,
                    "message":
                        "Task completed successfully",
                    "result":
                        task_result,
                }

            # ----------------------------------------------------
            # 失败
            # ----------------------------------------------------

            if is_failed_state(
                state
            ):
                message = (
                    payload.get(
                        "message"
                    )
                    or
                    f"Task ended with state={state}"
                )

                task_result = (
                    self.load_task_result(
                        task_id
                    )
                )

                self.set_record(
                    task_id,
                    status="failed",
                    stage=state.lower(),
                    message=message,
                    result=task_result,
                )

                self.get_logger().error(
                    "同步局放任务失败："
                    f"task_id={task_id}, "
                    f"state={state}, "
                    f"message={message}"
                )

                self.get_logger().info(
                    "================================================"
                )

                return {
                    "success": False,
                    "http_code": 500,
                    "message": message,
                    "result": task_result,
                }

            time.sleep(
                self.poll_interval
            )

        # --------------------------------------------------------
        # HTTP同步等待超时
        # 不主动中止机器人，避免在返回阶段强制取消。
        # --------------------------------------------------------

        message = (
            "HTTP synchronous wait timeout "
            f"after {self.task_timeout:.1f}s; "
            "robot task may still be running"
        )

        self.set_record(
            task_id,
            status="failed",
            stage="http_timeout",
            message=message,
        )

        self.get_logger().error(
            message
        )

        return {
            "success": False,
            "http_code": 504,
            "message": message,
        }


class HttpServer(
    ThreadingHTTPServer
):
    allow_reuse_address = True
    daemon_threads = True


def make_handler(
    bridge,
):
    execution_lock = (
        threading.Lock()
    )

    class Handler(
        BaseHTTPRequestHandler
    ):
        server_version = (
            "LeftProbeSyncHTTP/2.0"
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
            status_code,
            payload,
        ):
            body = json.dumps(
                payload,
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode(
                "utf-8"
            )

            try:
                self.send_response(
                    status_code
                )

                self.send_header(
                    "Content-Type",
                    (
                        "application/json; "
                        "charset=utf-8"
                    ),
                )

                self.send_header(
                    "Content-Length",
                    str(
                        len(body)
                    ),
                )

                self.send_header(
                    "Cache-Control",
                    "no-store",
                )

                self.send_header(
                    "Connection",
                    "close",
                )

                self.end_headers()

                self.wfile.write(
                    body
                )

            except (
                BrokenPipeError,
                ConnectionResetError,
            ):
                bridge.get_logger().warning(
                    "HTTP客户端在任务完成前"
                    "已经断开连接"
                )

        def read_json_body(
            self,
        ):
            try:
                length = int(
                    self.headers.get(
                        "Content-Length",
                        "0",
                    )
                )

            except Exception:
                length = 0

            if length <= 0:
                return {}

            if length > 1024 * 1024:
                raise ValueError(
                    "request_body_too_large"
                )

            raw = self.rfile.read(
                length
            )

            if not raw:
                return {}

            return json.loads(
                raw.decode(
                    "utf-8"
                )
            )

        # ========================================================
        # POST /start_task
        #
        # Java真实格式：
        #
        # {
        #   "task_type":"detection",
        #   "task_id":"..."
        # }
        #
        # 现在同步等待整个机器人任务结束。
        # ========================================================

        def do_POST(self):
            path = urlparse(
                self.path
            ).path.rstrip("/")

            if path != "/start_task":
                self.send_json(
                    404,
                    {
                        "status":
                            "failed",

                        "message":
                            "Not found",
                    },
                )
                return

            try:
                payload = (
                    self.read_json_body()
                )

            except Exception as exc:
                self.send_json(
                    400,
                    {
                        "status":
                            "failed",

                        "message":
                            f"Invalid JSON: {exc}",
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

            detection_type = str(
                payload.get(
                    "detection_type",
                    "all",
                )
            ).strip()

            if task_type != "detection":
                self.send_json(
                    400,
                    {
                        "task_id":
                            task_id,

                        "status":
                            "failed",

                        "type":
                            task_type,

                        "message":
                            (
                                "This service only "
                                "handles detection"
                            ),
                    },
                )
                return

            if not task_id:
                self.send_json(
                    400,
                    {
                        "status":
                            "failed",

                        "type":
                            "detection",

                        "message":
                            "task_id is required",
                    },
                )
                return

            if (
                detection_type
                not in
                VALID_DETECTION_TYPES
            ):
                self.send_json(
                    400,
                    {
                        "task_id":
                            task_id,

                        "status":
                            "failed",

                        "type":
                            "detection",

                        "message":
                            (
                                "invalid "
                                "detection_type"
                            ),
                    },
                )
                return

            # ----------------------------------------------------
            # 相同task_id重复请求：
            # 已经成功/失败则直接返回历史结果，不重复动机器人。
            # ----------------------------------------------------

            old = bridge.get_record(
                task_id
            )

            if old is not None:

                if (
                    old.get("status")
                    == "success"
                ):
                    self.send_json(
                        200,
                        old,
                    )
                    return

                if (
                    old.get("status")
                    == "failed"
                ):
                    self.send_json(
                        500,
                        old,
                    )
                    return

                if (
                    old.get("status")
                    == "executing"
                ):
                    self.send_json(
                        409,
                        {
                            "task_id":
                                task_id,

                            "status":
                                "executing",

                            "type":
                                "detection",

                            "message":
                                (
                                    "Task already "
                                    "executing"
                                ),
                        },
                    )
                    return

            # ----------------------------------------------------
            # 左臂只允许一个任务
            # ----------------------------------------------------

            if not execution_lock.acquire(
                blocking=False
            ):
                self.send_json(
                    409,
                    {
                        "task_id":
                            task_id,

                        "status":
                            "failed",

                        "type":
                            "detection",

                        "message":
                            (
                                "Another detection "
                                "task is executing"
                            ),
                    },
                )
                return

            try:
                result = (
                    bridge.execute_detection_sync(
                        task_id,
                        detection_type,
                    )
                )

            except Exception as exc:
                bridge.get_logger().exception(
                    "同步局放HTTP执行异常："
                    f"{exc}"
                )

                result = {
                    "success":
                        False,

                    "http_code":
                        500,

                    "message":
                        (
                            f"{type(exc).__name__}: "
                            f"{exc}"
                        ),
                }

                bridge.set_record(
                    task_id,
                    status="failed",
                    stage="http_exception",
                    message=result[
                        "message"
                    ],
                )

            finally:
                execution_lock.release()

            if result["success"]:
                response = {
                    "task_id":
                        task_id,

                    "status":
                        "success",

                    "type":
                        "detection",

                    "detection_type":
                        detection_type,

                    "message":
                        result.get(
                            "message",
                            "success",
                        ),
                }

                if (
                    result.get(
                        "result"
                    )
                    is not None
                ):
                    response["result"] = (
                        result["result"]
                    )

                self.send_json(
                    200,
                    response,
                )

            else:
                response = {
                    "task_id":
                        task_id,

                    "status":
                        "failed",

                    "type":
                        "detection",

                    "detection_type":
                        detection_type,

                    "message":
                        result.get(
                            "message",
                            "failed",
                        ),
                }

                if (
                    result.get(
                        "result"
                    )
                    is not None
                ):
                    response["result"] = (
                        result["result"]
                    )

                self.send_json(
                    int(
                        result.get(
                            "http_code",
                            500,
                        )
                    ),
                    response,
                )

        # ========================================================
        # GET /health
        # GET /status?task_id=...
        # ========================================================

        def do_GET(self):
            parsed = urlparse(
                self.path
            )

            path = (
                parsed.path.rstrip("/")
            )

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
                            "status":
                                "failed",

                            "message":
                                (
                                    "task_id "
                                    "is required"
                                ),
                        },
                    )
                    return

                record = (
                    bridge.get_record(
                        task_id
                    )
                )

                if record is None:
                    self.send_json(
                        200,
                        {
                            "task_id":
                                task_id,

                            "status":
                                "idle",

                            "type":
                                "detection",
                        },
                    )
                    return

                self.send_json(
                    200,
                    record,
                )
                return

            self.send_json(
                404,
                {
                    "status":
                        "failed",

                    "message":
                        "Not found",
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

    parser.add_argument(
        "--task-timeout",
        type=float,
        default=360.0,
    )

    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.25,
    )

    args = parser.parse_args()

    rclpy.init()

    bridge = LeftProbeHttpBridge(
        workspace=args.workspace,
        task_timeout=args.task_timeout,
        poll_interval=args.poll_interval,
    )

    # 先真正绑定端口。
    # 成功后再启动ROS executor，
    # 避免端口占用时产生之前的abort问题。
    try:
        server = HttpServer(
            (
                args.host,
                args.port,
            ),
            make_handler(
                bridge
            ),
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
        name="left_probe_http_ros",
    )

    ros_thread.start()

    bridge.get_logger().info(
        "================================================"
    )

    bridge.get_logger().info(
        "左臂局放同步 HTTP 接口已启动"
    )

    bridge.get_logger().info(
        f"监听：http://"
        f"{args.host}:"
        f"{args.port}"
    )

    bridge.get_logger().info(
        "POST /start_task：同步等待完整任务"
    )

    bridge.get_logger().info(
        "Java未传detection_type时：all"
    )

    bridge.get_logger().info(
        "SUCCESS -> HTTP 200"
    )

    bridge.get_logger().info(
        "FAILED  -> HTTP 4xx/5xx"
    )

    bridge.get_logger().info(
        f"同步等待超时："
        f"{args.task_timeout:.1f}s"
    )

    bridge.get_logger().info(
        "================================================"
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
