#!/usr/bin/env python3

import argparse
import collections
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


class D435Watchdog(Node):

    def __init__(self, args):
        super().__init__("left_d435_watchdog")

        self.args = args

        self.lock = threading.RLock()

        self.started_at = time.monotonic()
        self.last_frame_at = None
        self.last_frame_seq = 0

        self.state = "STARTING"
        self.recovery_in_progress = False
        self.recovery_started_at = None
        self.hw_reset_done = False

        self.restart_times = collections.deque()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Image,
            args.depth_topic,
            self.depth_callback,
            qos,
        )

        self.healthy_pub = self.create_publisher(
            Bool,
            "/left_probe/camera_healthy",
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            "/left_probe/camera_state",
            10,
        )

        self.hw_reset_client = self.create_client(
            Empty,
            args.hw_reset_service,
        )

        self.timer = self.create_timer(
            0.25,
            self.tick,
        )

        self.get_logger().info(
            "左D435 watchdog启动"
        )
        self.get_logger().info(
            f"depth_topic={args.depth_topic}"
        )
        self.get_logger().info(
            f"stale_timeout={args.stale_timeout:.1f}s"
        )
        self.get_logger().info(
            f"startup_grace={args.startup_grace:.1f}s"
        )

    # ============================================================
    # raw depth heartbeat
    # ============================================================

    def depth_callback(self, msg):
        del msg

        now = time.monotonic()

        with self.lock:
            self.last_frame_at = now
            self.last_frame_seq += 1

            if self.recovery_in_progress:
                self.get_logger().info(
                    "[RECOVERED] raw depth重新出现"
                )

            self.recovery_in_progress = False
            self.recovery_started_at = None
            self.hw_reset_done = False

            self.state = "HEALTHY"

    # ============================================================
    # status
    # ============================================================

    def publish_state(self):
        with self.lock:
            state = self.state

        healthy = (
            state == "HEALTHY"
        )

        self.healthy_pub.publish(
            Bool(data=healthy)
        )

        self.state_pub.publish(
            String(data=state)
        )

    # ============================================================
    # hw reset
    # ============================================================

    def request_hw_reset(self):
        if not self.hw_reset_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().error(
                "[HW RESET] 服务不可用"
            )
            return False

        self.get_logger().warning(
            "[HW RESET] 调用 "
            + self.args.hw_reset_service
        )

        future = self.hw_reset_client.call_async(
            Empty.Request()
        )

        # 不阻塞executor等待结果。
        future.add_done_callback(
            self.hw_reset_done_callback
        )

        return True

    def hw_reset_done_callback(self, future):
        try:
            future.result()

            self.get_logger().warning(
                "[HW RESET] 服务调用完成，"
                "等待D435重新枚举和恢复出帧"
            )

        except Exception as exc:
            self.get_logger().error(
                "[HW RESET ERROR] "
                f"{type(exc).__name__}: {exc}"
            )

    # ============================================================
    # process restart
    # ============================================================

    @staticmethod
    def get_matching_pids(pattern):
        try:
            out = subprocess.check_output(
                [
                    "pgrep",
                    "-f",
                    pattern,
                ],
                text=True,
            )

        except subprocess.CalledProcessError:
            return []

        result = []

        for item in out.split():
            try:
                pid = int(item)
            except Exception:
                continue

            if pid != os.getpid():
                result.append(pid)

        return result

    def terminate_pattern(
        self,
        pattern,
        name,
    ):
        pids = self.get_matching_pids(
            pattern
        )

        for pid in pids:
            self.get_logger().warning(
                f"[CAMERA RESTART] TERM {name} PID={pid}"
            )

            try:
                os.kill(
                    pid,
                    signal.SIGTERM,
                )
            except ProcessLookupError:
                pass

        return pids

    def kill_pattern(
        self,
        pattern,
        name,
    ):
        pids = self.get_matching_pids(
            pattern
        )

        for pid in pids:
            self.get_logger().warning(
                f"[CAMERA RESTART] KILL {name} PID={pid}"
            )

            try:
                os.kill(
                    pid,
                    signal.SIGKILL,
                )
            except ProcessLookupError:
                pass

    def restart_camera_process(self):
        now = time.monotonic()

        while (
            self.restart_times
            and
            now - self.restart_times[0]
            > self.args.restart_window
        ):
            self.restart_times.popleft()

        if (
            len(self.restart_times)
            >= self.args.max_restarts
        ):
            with self.lock:
                self.state = "DEGRADED"

            self.get_logger().error(
                "[DEGRADED] "
                f"{self.args.restart_window:.0f}s内已经重启"
                f"{len(self.restart_times)}次，"
                "停止自动重启"
            )

            return False

        self.restart_times.append(
            now
        )

        self.get_logger().warning(
            "[CAMERA RESTART] hw_reset后仍无raw depth，"
            "重启左D435 ROS进程"
        )

        launch_pattern = (
            "ros2 launch realsense2_camera rs_launch.py"
            ".*camera_namespace:=left_probe"
            ".*camera_name:=d435"
        )

        node_pattern = (
            "realsense2_camera_node"
            ".*__node:=d435"
            ".*__ns:=/left_probe"
        )

        self.terminate_pattern(
            launch_pattern,
            "ros2-launch",
        )

        self.terminate_pattern(
            node_pattern,
            "realsense-node",
        )

        time.sleep(2.0)

        self.kill_pattern(
            launch_pattern,
            "ros2-launch",
        )

        self.kill_pattern(
            node_pattern,
            "realsense-node",
        )

        time.sleep(1.0)

        script = (
            Path(self.args.workspace)
            / "start_left_d435.sh"
        )

        if not script.exists():
            self.get_logger().error(
                f"[CAMERA RESTART] 找不到 {script}"
            )
            return False

        log_file = (
            Path(self.args.workspace)
            / "left_probe_service"
            / "logs"
            / "watchdog_camera_restart.log"
        )

        log_file.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        log = open(
            log_file,
            "a",
            buffering=1,
            encoding="utf-8",
        )

        log.write(
            "\n\n"
            "============================================\n"
            f"WATCHDOG RESTART {time.strftime('%F %T')}\n"
            "============================================\n"
        )

        subprocess.Popen(
            [
                "bash",
                str(script),
            ],
            cwd=self.args.workspace,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=os.environ.copy(),
        )

        with self.lock:
            self.state = "RESTARTING"
            self.recovery_started_at = (
                time.monotonic()
            )

        return True

    # ============================================================
    # watchdog
    # ============================================================

    def tick(self):
        now = time.monotonic()

        with self.lock:
            last_frame_at = self.last_frame_at
            state = self.state
            recovering = self.recovery_in_progress
            recovery_started = self.recovery_started_at
            hw_reset_done = self.hw_reset_done

        # 启动宽限期
        if last_frame_at is None:

            if (
                now - self.started_at
                < self.args.startup_grace
            ):
                with self.lock:
                    self.state = "STARTING"

                self.publish_state()
                return

            frame_age = float("inf")

        else:
            frame_age = (
                now - last_frame_at
            )

        # 正常
        if (
            frame_age
            <= self.args.stale_timeout
        ):
            with self.lock:
                self.state = "HEALTHY"

            self.publish_state()
            return

        # 已熔断
        if state == "DEGRADED":
            self.publish_state()
            return

        # 第一次发现断流
        if not recovering:
            with self.lock:
                self.state = "LOST"
                self.recovery_in_progress = True
                self.recovery_started_at = now
                self.hw_reset_done = True

            self.get_logger().error(
                "[CAMERA LOST] "
                f"raw depth已有 {frame_age:.2f}s 未更新"
            )

            self.request_hw_reset()

            self.publish_state()
            return

        # hw_reset后等待恢复
        if (
            recovery_started is not None
            and
            hw_reset_done
            and
            now - recovery_started
            >= self.args.hw_reset_grace
        ):
            with self.lock:
                self.hw_reset_done = False
                self.recovery_started_at = now
                self.state = "RESTARTING"

            self.restart_camera_process()

        # ROS进程重启后仍没恢复
        elif (
            state == "RESTARTING"
            and
            recovery_started is not None
            and
            now - recovery_started
            >= self.args.restart_grace
        ):
            self.get_logger().error(
                "[CAMERA RESTART FAILED] "
                "ROS进程重启后仍无raw depth"
            )

            # 下一周期允许重新进行一次hw_reset。
            with self.lock:
                self.recovery_in_progress = False
                self.state = "LOST"

        self.publish_state()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--workspace",
        default=str(
            Path.home() / "ros2_ws"
        ),
    )

    parser.add_argument(
        "--depth-topic",
        default=(
            "/left_probe/d435/"
            "depth/image_rect_raw"
        ),
    )

    parser.add_argument(
        "--hw-reset-service",
        default=(
            "/left_probe/d435/hw_reset"
        ),
    )

    parser.add_argument(
        "--startup-grace",
        type=float,
        default=20.0,
    )

    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--hw-reset-grace",
        type=float,
        default=12.0,
    )

    parser.add_argument(
        "--restart-grace",
        type=float,
        default=15.0,
    )

    parser.add_argument(
        "--restart-window",
        type=float,
        default=600.0,
    )

    parser.add_argument(
        "--max-restarts",
        type=int,
        default=3,
    )

    args = parser.parse_args()

    rclpy.init()

    node = D435Watchdog(
        args
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
