#!/usr/bin/env python3

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


class LeftProbeTaskServer(Node):

    def __init__(self, args):
        super().__init__("left_probe_task_server")

        self.args = args
        self.workspace = Path(args.workspace).expanduser()
        self.pose_file = Path(args.pose_file).expanduser()

        self.handoff_file = (
            self.workspace
            / "left_probe_service"
            / "run"
            / "pending_http_task.json"
        )

        self.handoff_file.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.external_task_id = ""
        self.detection_type = "all"

        self.initial_script = (
            self.workspace / "left_arm_ensure_initial_pose.py"
        )
        self.control_script = (
            self.workspace / "left_arm_depth_cycle.py"
        )

        self.log_root = (
            self.workspace
            / "left_probe_service"
            / "task_logs"
        )
        self.log_root.mkdir(parents=True, exist_ok=True)

        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.RLock()

        self.state = "IDLE"
        self.busy = False
        self.mode = ""
        self.task_id = ""
        self.message = "等待任务"
        self.last_line = ""
        self.log_file = ""
        self.started_at = ""
        self.finished_at = ""
        self.cancel_requested = False

        self.current_process: Optional[subprocess.Popen] = None
        self.worker_thread: Optional[threading.Thread] = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state_pub = self.create_publisher(
            String,
            "/left_probe/task/state",
            qos,
        )

        self.busy_pub = self.create_publisher(
            Bool,
            "/left_probe/task/busy",
            qos,
        )

        self.move_stop_pub = self.create_publisher(
            Empty,
            "/left_arm/rm_driver/move_stop_cmd",
            10,
        )

        self.start_srv = self.create_service(
            Trigger,
            "/left_probe/task/start",
            self.start_callback,
            callback_group=self.callback_group,
        )

        self.plan_srv = self.create_service(
            Trigger,
            "/left_probe/task/plan",
            self.plan_callback,
            callback_group=self.callback_group,
        )

        self.reset_srv = self.create_service(
            Trigger,
            "/left_probe/task/reset",
            self.reset_callback,
            callback_group=self.callback_group,
        )

        self.status_srv = self.create_service(
            Trigger,
            "/left_probe/task/status",
            self.status_callback,
            callback_group=self.callback_group,
        )

        self.cancel_srv = self.create_service(
            Trigger,
            "/left_probe/task/cancel",
            self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.create_timer(1.0, self.publish_status)

        self.publish_status()

        self.get_logger().info(
            "左臂探头常驻任务接口已启动"
        )

    def status_payload(self) -> Dict:
        with self.lock:
            process_pid = (
                self.current_process.pid
                if (
                    self.current_process is not None
                    and self.current_process.poll() is None
                )
                else None
            )

            return {
                "state": self.state,
                "busy": self.busy,
                "mode": self.mode,
                "task_id": self.task_id,
                "message": self.message,
                "last_line": self.last_line,
                "log_file": self.log_file,
                "started_at": self.started_at,
                "finished_at": self.finished_at,
                "process_pid": process_pid,
                "cancel_requested": self.cancel_requested,
            }

    def publish_status(self):
        payload = self.status_payload()

        state_msg = String()
        state_msg.data = json.dumps(
            payload,
            ensure_ascii=False,
        )
        self.state_pub.publish(state_msg)

        busy_msg = Bool()
        busy_msg.data = bool(payload["busy"])
        self.busy_pub.publish(busy_msg)

    def update_state(
        self,
        state: str,
        message: str,
    ):
        with self.lock:
            self.state = state
            self.message = message

        self.publish_status()

        self.get_logger().info(
            f"[{state}] {message}"
        )

    def finish_task(
        self,
        state: str,
        message: str,
    ):
        with self.lock:
            self.state = state
            self.message = message
            self.busy = False
            self.finished_at = datetime.now().isoformat(
                timespec="seconds"
            )
            self.current_process = None

        self.publish_status()

        self.get_logger().info(
            f"[{state}] {message}"
        )

    def dependencies_ready(self) -> Tuple[bool, str]:
        missing = []

        if not self.initial_script.exists():
            missing.append(str(self.initial_script))

        if not self.control_script.exists():
            missing.append(str(self.control_script))

        if not self.pose_file.exists():
            missing.append(str(self.pose_file))

        if missing:
            return (
                False,
                "缺少文件：" + "，".join(missing),
            )

        arm_canfd_ready = len(
            self.get_subscriptions_info_by_topic(
                "/left_arm/rm_driver/movej_canfd_cmd"
            )
        ) >= 1

        arm_state_ready = len(
            self.get_subscriptions_info_by_topic(
                "/left_arm/rm_driver/"
                "get_current_arm_state_cmd"
            )
        ) >= 1

        arm_movej_ready = len(
            self.get_subscriptions_info_by_topic(
                "/left_arm/rm_driver/movej_cmd"
            )
        ) >= 1

        depth_ready = len(
            self.get_publishers_info_by_topic(
                "/left_probe/near_distance_m"
            )
        ) >= 1

        valid_ready = len(
            self.get_publishers_info_by_topic(
                "/left_probe/distance_valid"
            )
        ) >= 1

        if not arm_canfd_ready:
            return False, "左臂CANFD接口未就绪"

        if not arm_state_ready:
            return False, "左臂状态接口未就绪"

        if not arm_movej_ready:
            return False, "左臂MoveJ接口未就绪"

        if not depth_ready:
            return False, "D435距离话题未就绪"

        if not valid_ready:
            return False, "D435有效状态话题未就绪"

        return True, "基础服务正常"

    def consume_http_handoff(self):
        """
        HTTP -> ROS Trigger参数交接。

        文件由HTTP服务原子写入，
        本函数读取后立即删除。
        """

        default = {
            "task_id": "",
            "detection_type": "all",
        }

        if not self.handoff_file.exists():
            return default

        try:
            payload = json.loads(
                self.handoff_file.read_text(
                    encoding="utf-8"
                )
            )

        finally:
            try:
                self.handoff_file.unlink()
            except FileNotFoundError:
                pass

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
        ).strip().lower()

        if task_id and not re.fullmatch(
            r"[A-Za-z0-9_.-]{1,128}",
            task_id,
        ):
            raise ValueError(
                "task_id只允许"
                "A-Z/a-z/0-9/_/./-，"
                "长度1~128"
            )

        allowed = {
            "open_ultrasonic",
            "tev",
            "contact_ultrasonic",
            "uhf",
            "all",
        }

        if detection_type not in allowed:
            raise ValueError(
                "不支持的detection_type："
                f"{detection_type}"
            )

        return {
            "task_id":
                task_id,

            "detection_type":
                detection_type,
        }

    def start_job(
        self,
        mode: str,
    ) -> Tuple[bool, Dict]:
        ready, reason = self.dependencies_ready()

        if not ready:
            return False, {
                "accepted": False,
                "reason": reason,
            }

        with self.lock:
            if self.busy:
                return False, {
                    "accepted": False,
                    "reason": "已有任务正在执行",
                    "current": self.status_payload(),
                }

            timestamp = datetime.now().strftime(
                "%Y%m%d_%H%M%S_%f"
            )

            if (
                mode == "execute"
                and self.external_task_id
            ):
                self.task_id = (
                    self.external_task_id
                )
            else:
                self.task_id = (
                    f"{mode}_{timestamp}"
                )

            if mode != "execute":
                self.detection_type = "all"

            self.mode = mode
            self.state = "QUEUED"
            self.message = "任务已进入队列"
            self.last_line = ""
            self.started_at = datetime.now().isoformat(
                timespec="seconds"
            )
            self.finished_at = ""
            self.cancel_requested = False
            self.busy = True

            task_dir = self.log_root / self.task_id
            task_dir.mkdir(parents=True, exist_ok=True)
            self.log_file = str(task_dir)

            self.worker_thread = threading.Thread(
                target=self.worker,
                args=(mode, task_dir),
                daemon=True,
            )
            self.worker_thread.start()

        self.publish_status()

        return True, {
            "accepted": True,
            "task_id": self.task_id,
            "mode": mode,
            "detection_type": self.detection_type,
            "status_service": "/left_probe/task/status",
            "state_topic": "/left_probe/task/state",
        }

    def start_callback(self, request, response):
        del request

        try:
            handoff = self.consume_http_handoff()

        except Exception as exc:
            response.success = False
            response.message = json.dumps(
                {
                    "accepted": False,
                    "reason": (
                        "HTTP任务参数错误："
                        f"{exc}"
                    ),
                },
                ensure_ascii=False,
            )
            return response

        with self.lock:
            self.external_task_id = (
                handoff["task_id"]
            )

            self.detection_type = (
                handoff["detection_type"]
            )

        accepted, payload = self.start_job("execute")

        response.success = accepted
        response.message = json.dumps(
            payload,
            ensure_ascii=False,
        )
        return response

    def plan_callback(self, request, response):
        del request

        accepted, payload = self.start_job("plan")

        response.success = accepted
        response.message = json.dumps(
            payload,
            ensure_ascii=False,
        )
        return response

    def reset_callback(self, request, response):
        del request

        accepted, payload = self.start_job("reset")

        response.success = accepted
        response.message = json.dumps(
            payload,
            ensure_ascii=False,
        )
        return response

    def status_callback(self, request, response):
        del request

        response.success = True
        response.message = json.dumps(
            self.status_payload(),
            ensure_ascii=False,
        )
        return response

    def cancel_callback(self, request, response):
        del request

        with self.lock:
            if not self.busy:
                response.success = False
                response.message = json.dumps(
                    {
                        "accepted": False,
                        "reason": "当前没有运行中的任务",
                    },
                    ensure_ascii=False,
                )
                return response

            self.cancel_requested = True
            process = self.current_process

        self.update_state(
            "CANCEL_REQUESTED",
            "收到取消指令，正在停止机械臂任务",
        )

        # 先发送机械臂停止命令
        for _ in range(3):
            self.move_stop_pub.publish(Empty())

        # 再终止当前子进程组
        if (
            process is not None
            and process.poll() is None
        ):
            try:
                os.killpg(
                    process.pid,
                    signal.SIGINT,
                )
            except ProcessLookupError:
                pass

        response.success = True
        response.message = json.dumps(
            {
                "accepted": True,
                "task_id": self.task_id,
                "state": "CANCEL_REQUESTED",
            },
            ensure_ascii=False,
        )
        return response

    def initial_command(self):
        return [
            sys.executable,
            "-u",
            str(self.initial_script),
            "--pose-file",
            str(self.pose_file),
            "--move",
            "--speed",
            str(self.args.restore_speed),
            "--timeout",
            str(self.args.restore_timeout),
            "--joint-tolerance-deg",
            "0.50",
            "--verify-tolerance-deg",
            "0.60",
        ]

    def control_command(self, mode: str):
        command = [
            sys.executable,
            "-u",
            str(self.control_script),
            "--stop-depth",
            str(self.args.stop_depth),
            "--hard-min-depth",
            str(self.args.hard_min_depth),
            "--slow-depth",
            str(self.args.slow_depth),
            "--max-distance",
            str(self.args.max_distance),
            "--reach-margin",
            str(self.args.reach_margin),
            "--path-reserve",
            str(self.args.path_reserve),
            "--depth-motion-scale",
            str(self.args.depth_motion_scale),
            "--fast-speed",
            str(self.args.fast_speed),
            "--slow-speed",
            str(self.args.slow_speed),
            "--return-speed",
            str(self.args.return_speed),
            "--dwell",
            str(self.args.dwell),
            "--rate",
            str(self.args.rate),
            "--key-step",
            str(self.args.key_step),
        ]

        if mode == "execute":
            command.append("--execute")

        return command

    def parse_control_line(
        self,
        line: str,
        flags: Dict,
    ):
        stripped = line.strip()

        with self.lock:
            self.last_line = stripped

        if not stripped:
            return

        if stripped.startswith("停止原因："):
            stop_reason = stripped.split("：", 1)[1].strip()
            flags["stop_reason"] = stop_reason

            if stop_reason != "stop_depth":
                self.update_state(
                    "ABORTING",
                    (
                        "未达到正常停止距离，"
                        f"停止原因={stop_reason}；"
                        "正在安全返回"
                    ),
                )

        if "[TASK REJECTED]" in stripped:
            flags["rejected"] = True

            self.update_state(
                "REJECTED",
                stripped,
            )

        elif "任务可达性判断" in stripped:
            self.update_state(
                "VALIDATING",
                "正在判断目标距离和机械臂量程",
            )

        elif "机械臂轨迹规划" in stripped:
            self.update_state(
                "PLANNING",
                "正在生成机械臂前伸轨迹",
            )

        elif stripped.startswith("depth="):
            if self.state != "APPROACHING":
                self.update_state(
                    "APPROACHING",
                    "机械臂正在根据D435距离前伸",
                )

        elif "[DWELL]" in stripped:
            self.update_state(
                "DWELL",
                "探头已到位，正在保持检测位置",
            )

        elif "[G300 START]" in stripped:
            self.update_state(
                "G300_DETECTING",
                (
                    "G300正在执行"
                    f"{self.detection_type}检测"
                ),
            )

        elif "[G300 SUCCESS]" in stripped:
            flags["g300_success"] = True

            self.update_state(
                "G300_COMPLETE",
                stripped,
            )

        elif "[G300 ERROR]" in stripped:
            flags["g300_error"] = True

            self.update_state(
                "G300_ERROR",
                stripped,
            )

        elif "[RETURN]" in stripped:
            self.update_state(
                "RETURNING",
                "机械臂正在沿原轨迹返回",
            )

        elif "循环完成" in stripped:
            flags["cycle_complete"] = True

            self.update_state(
                "VERIFYING",
                "运动循环完成，正在校验返回位置",
            )

        elif "[PLAN ONLY]" in stripped:
            flags["plan_complete"] = True

        if "[TASK INCOMPLETE]" in stripped:
            flags["task_incomplete"] = True

        if "[TASK SUCCESS]" in stripped:
            flags["task_success"] = True

    def run_process(
        self,
        command,
        log_path: Path,
        phase: str,
    ) -> Tuple[int, Dict]:
        flags = {
            "rejected": False,
            "cycle_complete": False,
            "plan_complete": False,
            "task_incomplete": False,
            "task_success": False,
            "g300_success": False,
            "g300_error": False,
            "stop_reason": "",
        }

        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"

        environment[
            "LEFT_PROBE_TASK_ID"
        ] = self.task_id

        environment[
            "LEFT_PROBE_DETECTION_TYPE"
        ] = self.detection_type

        environment[
            "LEFT_PROBE_G300_SCRIPT"
        ] = str(
            self.workspace
            / "g300_full_acquire.py"
        )

        environment[
            "LEFT_PROBE_G300_PORT"
        ] = "/dev/ttyUSB0"

        environment[
            "LEFT_PROBE_G300_BAUD"
        ] = "115200"

        environment[
            "LEFT_PROBE_G300_ID"
        ] = "2"

        environment[
            "LEFT_PROBE_G300_CYCLES"
        ] = "20"

        environment[
            "LEFT_PROBE_G300_OUTPUT_ROOT"
        ] = str(
            log_path.parent
            / "g300"
        )

        environment[
            "LEFT_PROBE_TASK_RESULT_FILE"
        ] = str(
            log_path.parent
            / "task_result.json"
        )

        with log_path.open(
            "w",
            encoding="utf-8",
            buffering=1,
        ) as log_file:

            log_file.write(
                "COMMAND: "
                + " ".join(command)
                + "\n\n"
            )

            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                env=environment,
                start_new_session=True,
            )

            with self.lock:
                self.current_process = process

            assert process.stdout is not None

            for line in process.stdout:
                print(line, end="")
                log_file.write(line)

                with self.lock:
                    self.last_line = line.strip()

                if phase == "control":
                    self.parse_control_line(
                        line,
                        flags,
                    )

            return_code = process.wait()

            with self.lock:
                if self.current_process is process:
                    self.current_process = None

        return return_code, flags

    def worker(
        self,
        mode: str,
        task_dir: Path,
    ):
        try:
            # --------------------------------------------------------
            # 1. 每次任务开始前检查并复位初始位置
            # --------------------------------------------------------
            self.update_state(
                "RESTORING_INITIAL",
                "正在检查并复位到保存的初始位姿",
            )

            initial_rc, _ = self.run_process(
                self.initial_command(),
                task_dir / "initial_pose.log",
                phase="initial",
            )

            with self.lock:
                canceled = self.cancel_requested

            if canceled:
                self.finish_task(
                    "CANCELED",
                    "任务已取消",
                )
                return

            if initial_rc != 0:
                self.finish_task(
                    "FAILED",
                    (
                        "初始位置复位失败，"
                        f"退出码={initial_rc}"
                    ),
                )
                return

            # 仅复位任务到这里结束
            if mode == "reset":
                self.finish_task(
                    "SUCCESS",
                    "机械臂已复位到初始位置",
                )
                return

            # --------------------------------------------------------
            # 2. 运行规划或完整运动程序
            # flags只在run_process返回后才允许使用
            # --------------------------------------------------------
            if mode == "plan":
                self.update_state(
                    "PLANNING",
                    "正在执行只规划检查",
                )
            else:
                self.update_state(
                    "VALIDATING",
                    "正在读取目标距离并准备执行",
                )

            control_rc, flags = self.run_process(
                self.control_command(mode),
                task_dir / "control.log",
                phase="control",
            )

            with self.lock:
                canceled = self.cancel_requested

            if canceled:
                self.finish_task(
                    "CANCELED",
                    "任务已取消，机械臂已发送停止命令",
                )
                return

            if flags.get("rejected", False):
                self.finish_task(
                    "REJECTED",
                    (
                        self.last_line
                        or "目标不满足执行条件"
                    ),
                )
                return

            if control_rc != 0:
                self.finish_task(
                    "FAILED",
                    (
                        "控制程序异常退出，"
                        f"退出码={control_rc}"
                    ),
                )
                return

            # --------------------------------------------------------
            # 3. 只规划模式结果
            # --------------------------------------------------------
            if mode == "plan":
                if flags.get("plan_complete", False):
                    self.finish_task(
                        "PLAN_COMPLETE",
                        (
                            "可达性检查和轨迹规划完成，"
                            "机械臂未执行前伸"
                        ),
                    )
                else:
                    self.finish_task(
                        "FAILED",
                        (
                            "规划程序已经退出，"
                            "但没有检测到规划完成标志"
                        ),
                    )

                return

            # --------------------------------------------------------
            # 4. 完整任务结果判断
            # --------------------------------------------------------
            stop_reason = flags.get(
                "stop_reason",
                "",
            )

            incomplete = (
                flags.get("task_incomplete", False)
                or (
                    bool(stop_reason)
                    and stop_reason != "stop_depth"
                )
            )

            if incomplete:
                self.finish_task(
                    "INCOMPLETE_RETURNED",
                    (
                        "未达到正常停止深度；"
                        f"停止原因={stop_reason or 'unknown'}；"
                        "机械臂已安全返回初始位置"
                    ),
                )
                return

            if flags.get(
                "g300_error",
                False,
            ):
                self.finish_task(
                    "FAILED",
                    (
                        "G300检测失败，"
                        "但机械臂已安全返回初始位置"
                    ),
                )
                return

            complete_success = (
                flags.get("cycle_complete", False)
                and stop_reason == "stop_depth"
                and flags.get("g300_success", False)
                and flags.get("task_success", False)
            )

            if complete_success:
                self.finish_task(
                    "SUCCESS",
                    (
                        "探头达到设定停止距离，"
                        "完成G300检测并返回初始位置"
                    ),
                )
                return

            self.finish_task(
                "FAILED",
                (
                    "控制程序退出，但完整任务条件不满足："
                    f"stop_reason={stop_reason or 'missing'}，"
                    f"cycle_complete="
                    f"{flags.get('cycle_complete', False)}，"
                    f"task_success="
                    f"{flags.get('task_success', False)}"
                ),
            )

        except Exception as exc:
            self.finish_task(
                "FAILED",
                f"{type(exc).__name__}: {exc}",
            )


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--workspace",
        default=str(Path.home() / "ros2_ws"),
    )

    parser.add_argument(
        "--pose-file",
        default=str(
            Path.home()
            / "ros2_ws"
            / "left_arm_longstroke_start_retracted_150mm.json"
        ),
    )

    parser.add_argument(
        "--restore-speed",
        type=int,
        default=5,
    )

    parser.add_argument(
        "--restore-timeout",
        type=float,
        default=60.0,
    )

    parser.add_argument("--stop-depth", type=float, default=0.140)
    parser.add_argument("--hard-min-depth", type=float, default=0.090)
    parser.add_argument("--slow-depth", type=float, default=0.200)

    parser.add_argument("--max-distance", type=float, default=0.350)
    parser.add_argument("--reach-margin", type=float, default=0.010)
    parser.add_argument("--path-reserve", type=float, default=0.050)
    parser.add_argument(
        "--depth-motion-scale",
        type=float,
        default=0.80,
    )

    parser.add_argument("--fast-speed", type=float, default=0.080)
    parser.add_argument("--slow-speed", type=float, default=0.015)
    parser.add_argument("--return-speed", type=float, default=0.080)

    parser.add_argument("--dwell", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--key-step", type=float, default=0.0005)

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = LeftProbeTaskServer(args)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
