#!/usr/bin/env python3

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path
import math
import time
from typing import Optional

import rclpy
from std_msgs.msg import Bool, Float32

from left_arm_smooth_canfd_forward import (
    SmoothForward,
    arm_error,
    create_algo,
    interpolate_path,
    plan_path,
    singularity_reason,
    smoothstep5,
    wait_until,
)


DISTANCE_TOPIC = "/left_probe/near_distance_m"
VALID_TOPIC = "/left_probe/distance_valid"
TOO_CLOSE_TOPIC = "/left_probe/too_close"


class LeftArmDepthCycle(SmoothForward):

    def __init__(self):
        super().__init__()

        self.depth_m: Optional[float] = None
        self.depth_valid = False
        self.too_close = False

        self.depth_seq = 0
        self.depth_stamp = 0.0

        self.create_subscription(
            Float32,
            DISTANCE_TOPIC,
            self.depth_callback,
            10,
        )

        self.create_subscription(
            Bool,
            VALID_TOPIC,
            self.valid_callback,
            10,
        )

        # 该话题不存在时不会影响运行
        self.create_subscription(
            Bool,
            TOO_CLOSE_TOPIC,
            self.too_close_callback,
            10,
        )

    def depth_callback(self, msg):
        value = float(msg.data)

        if math.isfinite(value):
            self.depth_m = value
            self.depth_seq += 1
            self.depth_stamp = time.monotonic()

    def valid_callback(self, msg):
        self.depth_valid = bool(msg.data)

    def too_close_callback(self, msg):
        self.too_close = bool(msg.data)

    def depth_is_fresh(self, max_age=0.30):
        return (
            self.depth_valid
            and self.depth_m is not None
            and time.monotonic() - self.depth_stamp <= max_age
        )

    def wait_for_depth(self, timeout=8.0):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.depth_is_fresh():
                return float(self.depth_m)

        raise RuntimeError("未收到有效D435深度数据")

    def hold_joint(
        self,
        q_deg,
        duration,
        rate,
        high_follow,
        hard_min_depth=None,
    ):
        period = 1.0 / rate
        count = max(1, int(round(duration * rate)))
        start = time.perf_counter()

        for index in range(count):
            self.publish_joint(q_deg, high_follow)
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.too_close:
                print("[HARD STOP] 测距节点报告too_close")
                return False

            if (
                hard_min_depth is not None
                and self.depth_is_fresh()
                and self.depth_m <= hard_min_depth
            ):
                print(
                    "[HARD STOP] "
                    f"深度={self.depth_m:.3f} m，"
                    f"达到硬极限{hard_min_depth:.3f} m"
                )
                return False

            wait_until(start + (index + 1) * period)

        return True

    def approach(
        self,
        key_progress,
        key_joints,
        max_distance,
        stop_depth,
        hard_min_depth,
        slow_depth,
        fast_speed,
        slow_speed,
        acceleration,
        deceleration,
        confirm_frames,
        depth_timeout,
        rate,
        high_follow,
    ):
        period = 1.0 / rate

        traveled = 0.0
        current_speed = 0.0
        current_q = list(key_joints[0])

        confirm_count = 0
        last_depth_seq = -1
        last_good_depth_time = time.monotonic()
        last_good_depth = None

        loop_index = 0
        print_time = 0.0
        max_lateness = 0.0

        self.hold_joint(
            current_q,
            duration=0.20,
            rate=rate,
            high_follow=high_follow,
        )

        start_time = time.perf_counter()

        while traveled < max_distance - 1e-9:
            deadline = start_time + loop_index * period
            wait_until(deadline)

            lateness = time.perf_counter() - deadline
            max_lateness = max(max_lateness, lateness)

            rclpy.spin_once(self, timeout_sec=0.0)

            if self.too_close:
                return (
                    traveled,
                    current_q,
                    "too_close",
                    max_lateness,
                )

            if self.depth_is_fresh():
                depth = float(self.depth_m)
                last_good_depth = depth
                last_good_depth_time = time.monotonic()

                # 硬极限：不等待连续帧
                if depth <= hard_min_depth:
                    return (
                        traveled,
                        current_q,
                        "hard_min_depth",
                        max_lateness,
                    )

                # 正常停止阈值：连续3个新深度帧确认
                if self.depth_seq != last_depth_seq:
                    last_depth_seq = self.depth_seq

                    if depth <= stop_depth:
                        confirm_count += 1

                        print(
                            f"[STOP CONFIRM] "
                            f"depth={depth:.6f} m  "
                            f"threshold={stop_depth:.6f} m  "
                            f"confirm={confirm_count}/{confirm_frames}",
                            flush=True,
                        )
                    else:
                        if confirm_count > 0:
                            print(
                                f"[STOP CONFIRM RESET] "
                                f"depth={depth:.6f} m > "
                                f"threshold={stop_depth:.6f} m",
                                flush=True,
                            )

                        confirm_count = 0

                # 第一次进入停止区后便冻结轨迹，
                # 防止等待确认帧期间继续前伸
                if confirm_count > 0:
                    current_speed = 0.0
                    self.publish_joint(current_q, high_follow)

                    if confirm_count >= confirm_frames:
                        return (
                            traveled,
                            current_q,
                            "stop_depth",
                            max_lateness,
                        )

                    loop_index += 1
                    continue

            else:
                # 深度短暂失效时，冻结当前位置
                current_speed = 0.0
                self.publish_joint(current_q, high_follow)

                lost_time = (
                    time.monotonic() - last_good_depth_time
                )

                # 接近目标后发生深度丢失，立即结束前伸
                if (
                    last_good_depth is not None
                    and last_good_depth <= slow_depth
                    and lost_time >= 0.15
                ):
                    return (
                        traveled,
                        current_q,
                        "depth_lost_near_target",
                        max_lateness,
                    )

                if lost_time >= depth_timeout:
                    return (
                        traveled,
                        current_q,
                        "depth_timeout",
                        max_lateness,
                    )

                loop_index += 1
                continue

            depth_margin = max(
                0.0,
                float(self.depth_m) - stop_depth,
            )

            path_margin = max(
                0.0,
                max_distance - traveled,
            )

            depth_braking_speed = math.sqrt(
                2.0 * deceleration * depth_margin
            )

            path_braking_speed = math.sqrt(
                2.0 * deceleration * path_margin
            )

            desired_speed = min(
                fast_speed,
                depth_braking_speed,
                path_braking_speed,
            )

            if self.depth_m <= slow_depth:
                desired_speed = min(
                    desired_speed,
                    slow_speed,
                )

            if current_speed < desired_speed:
                current_speed = min(
                    desired_speed,
                    current_speed + acceleration * period,
                )
            else:
                current_speed = max(
                    desired_speed,
                    current_speed - deceleration * period,
                )

            traveled = min(
                max_distance,
                traveled + current_speed * period,
            )

            progress = traveled / max_distance

            current_q = interpolate_path(
                progress,
                key_progress,
                key_joints,
            )

            self.publish_joint(current_q, high_follow)

            now = time.monotonic()

            if now - print_time >= 0.20:
                print(
                    f"depth={self.depth_m:.3f} m  "
                    f"travel={traveled * 1000:7.1f} mm  "
                    f"speed={current_speed * 1000:5.1f} mm/s  "
                    f"confirm={confirm_count}/{confirm_frames}",
                    flush=True,
                )
                print_time = now

            loop_index += 1

        return (
            traveled,
            current_q,
            "max_distance",
            max_lateness,
        )

    def return_to_start(
        self,
        key_progress,
        key_joints,
        traveled,
        max_distance,
        return_speed,
        rate,
        high_follow,
    ):
        start_progress = max(
            0.0,
            min(1.0, traveled / max_distance),
        )

        duration = max(
            1.0,
            traveled / return_speed,
        )

        samples = max(
            20,
            int(round(duration * rate)),
        )

        period = 1.0 / rate
        start_time = time.perf_counter()

        print(
            f"[RETURN] 原路收回{traveled * 1000:.1f} mm，"
            f"预计{duration:.2f} s"
        )

        for index in range(samples + 1):
            u = index / samples

            progress = start_progress * (
                1.0 - smoothstep5(u)
            )

            q = interpolate_path(
                progress,
                key_progress,
                key_joints,
            )

            wait_until(start_time + index * period)

            self.publish_joint(q, high_follow)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.hold_joint(
            key_joints[0],
            duration=1.20,
            rate=rate,
            high_follow=high_follow,
        )



def run_g300_detection_while_holding(
    node,
    q_deg,
    rate,
    high_follow,
    hard_min_depth,
):
    """
    在机械臂保持stop_q期间运行G300。

    G300作为独立子进程运行；
    当前线程持续调用hold_joint保持机械臂位置。
    """

    detection_type = os.environ.get(
        "LEFT_PROBE_DETECTION_TYPE",
        "all",
    ).strip()

    task_id = os.environ.get(
        "LEFT_PROBE_TASK_ID",
        "",
    ).strip()

    output_root_text = os.environ.get(
        "LEFT_PROBE_G300_OUTPUT_ROOT",
        "",
    ).strip()

    if output_root_text:
        output_root = Path(
            output_root_text
        ).expanduser()
    else:
        output_root = (
            Path.home()
            / "ros2_ws"
            / "g300_results"
        )

    output_root.mkdir(
        parents=True,
        exist_ok=True,
    )

    script = Path(
        os.environ.get(
            "LEFT_PROBE_G300_SCRIPT",
            str(
                Path.home()
                / "ros2_ws"
                / "g300_full_acquire.py"
            ),
        )
    ).expanduser()

    port = os.environ.get(
        "LEFT_PROBE_G300_PORT",
        "/dev/ttyUSB0",
    )

    baud = os.environ.get(
        "LEFT_PROBE_G300_BAUD",
        "115200",
    )

    slave_id = os.environ.get(
        "LEFT_PROBE_G300_ID",
        "2",
    )

    cycles = os.environ.get(
        "LEFT_PROBE_G300_CYCLES",
        "20",
    )

    cmd = [
        sys.executable,
        "-u",
        str(script),

        "--port",
        port,

        "--baud",
        str(baud),

        "--id",
        str(slave_id),

        "--cycles",
        str(cycles),

        "--channels",
        detection_type,

        "--output-root",
        str(output_root),
    ]

    print(
        "[G300 START] "
        f"task_id={task_id or '-'} "
        f"detection_type={detection_type}",
        flush=True,
    )

    print(
        "[G300 PATH] "
        f"{output_root}",
        flush=True,
    )

    start_time = time.time()

    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
    )

    output_lines = []

    def reader():
        assert process.stdout is not None

        for line in process.stdout:
            stripped = line.rstrip("\n")

            output_lines.append(
                stripped
            )

            # G300脚本已经是简洁模式，
            # 直接透传到机械臂任务日志。
            print(
                "[G300] "
                + stripped,
                flush=True,
            )

    reader_thread = threading.Thread(
        target=reader,
        daemon=True,
    )

    reader_thread.start()

    hold_ok = True
    hold_abort_reason = ""

    try:
        while process.poll() is None:

            # 0.20s一个保持窗口；
            # 内部仍按rate连续发送CANFD。
            ok = node.hold_joint(
                q_deg,
                duration=0.20,
                rate=rate,
                high_follow=high_follow,
                hard_min_depth=hard_min_depth,
            )

            if not ok:
                hold_ok = False
                hold_abort_reason = (
                    "机械臂保持期间触发"
                    "hard_min_depth"
                )

                print(
                    "[G300 HOLD ABORT] "
                    + hold_abort_reason,
                    flush=True,
                )

                try:
                    process.send_signal(
                        signal.SIGINT
                    )
                except Exception:
                    pass

                break

        try:
            rc = process.wait(
                timeout=10.0
            )

        except subprocess.TimeoutExpired:

            try:
                process.kill()
            except Exception:
                pass

            rc = process.wait()

    finally:
        reader_thread.join(
            timeout=2.0
        )

    if not hold_ok:
        return False, {
            "task_id": task_id,
            "detection_type":
                detection_type,
            "reason":
                hold_abort_reason,
        }

    if rc != 0:
        return False, {
            "task_id": task_id,
            "detection_type":
                detection_type,
            "reason":
                (
                    "G300采集程序异常退出，"
                    f"return_code={rc}"
                ),
        }

    # --------------------------------------------------------
    # 找本次刚生成的full_result.json
    # --------------------------------------------------------

    candidates = []

    for result_file in (
        output_root.glob(
            "*/full_result.json"
        )
    ):
        try:
            if (
                result_file.stat().st_mtime
                >= start_time - 2.0
            ):
                candidates.append(
                    result_file
                )
        except OSError:
            pass

    if not candidates:
        return False, {
            "task_id": task_id,
            "detection_type":
                detection_type,
            "reason":
                "G300完成但未找到full_result.json",
        }

    full_result_path = max(
        candidates,
        key=lambda p: p.stat().st_mtime,
    )

    try:
        payload = json.loads(
            full_result_path.read_text(
                encoding="utf-8"
            )
        )

    except Exception as exc:
        return False, {
            "task_id": task_id,
            "detection_type":
                detection_type,
            "result_file":
                str(full_result_path),
            "reason":
                (
                    "读取G300结果失败："
                    f"{exc}"
                ),
        }

    if not payload.get(
        "success",
        False,
    ):
        return False, {
            "task_id": task_id,
            "detection_type":
                detection_type,
            "result_file":
                str(full_result_path),
            "reason":
                "G300结果文件标记success=false",
        }

    channels = []

    for item in payload.get(
        "channels",
        [],
    ):
        channels.append({
            "name":
                item.get("name"),

            "name_cn":
                item.get("name_cn"),

            "success":
                item.get("success"),

            "detected":
                item.get("detected"),

            "discharge_type":
                item.get(
                    "final_discharge_type"
                ),

            "discharge_type_name":
                item.get(
                    "final_discharge_type_name"
                ),

            "basic_value":
                (
                    item.get(
                        "final_basic"
                    )
                    or {}
                ).get(
                    "basic_value"
                ),

            "pulse_count":
                (
                    item.get(
                        "final_basic"
                    )
                    or {}
                ).get(
                    "pulse_count"
                ),

            "intensity":
                (
                    item.get(
                        "intensity"
                    )
                    or {}
                ).get(
                    "name"
                ),
        })

    detected = any(
        bool(item.get("detected"))
        for item in channels
    )

    summary = {
        "task_id":
            task_id,

        "detection_type":
            detection_type,

        "success":
            True,

        "detected":
            detected,

        "channels":
            channels,

        "result_dir":
            str(
                full_result_path.parent
            ),

        "full_result":
            str(full_result_path),
    }

    print(
        "[G300 RESULT] "
        + json.dumps(
            summary,
            ensure_ascii=False,
        ),
        flush=True,
    )

    print(
        "[G300 SUCCESS] "
        + (
            "检测到放电"
            if detected
            else "检测完成，未检测到放电"
        ),
        flush=True,
    )

    return True, summary

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--stop-depth",
        type=float,
        default=0.140,
    )

    parser.add_argument(
        "--hard-min-depth",
        type=float,
        default=0.090,
    )

    parser.add_argument(
        "--slow-depth",
        type=float,
        default=0.200,
    )

    parser.add_argument(
        "--max-distance",
        type=float,
        default=0.500,
        help="机械臂绝对最大前伸距离，默认0.35m",
    )

    parser.add_argument(
        "--reach-margin",
        type=float,
        default=0.010,
        help=(
            "最大量程预留余量，默认0.01m；"
            "正常任务允许前伸距离为max-distance减去该值"
        ),
    )

    parser.add_argument(
        "--path-reserve",
        type=float,
        default=0.050,
        help=(
            "估算行程之外额外规划的轨迹长度，"
            "默认50mm；实际停止仍由D435深度决定"
        ),
    )


    parser.add_argument(
        "--depth-motion-scale",
        type=float,
        default=0.80,
        help=(
            "D435距离变化/机械臂前伸距离换算系数；"
            "本次实测约0.832，默认保守取0.80"
        ),
    )

    parser.add_argument(
        "--fast-speed",
        type=float,
        default=0.080,
    )

    parser.add_argument(
        "--slow-speed",
        type=float,
        default=0.015,
    )

    parser.add_argument(
        "--return-speed",
        type=float,
        default=0.080,
    )

    parser.add_argument(
        "--acceleration",
        type=float,
        default=0.25,
    )

    parser.add_argument(
        "--deceleration",
        type=float,
        default=0.35,
    )

    parser.add_argument(
        "--confirm-frames",
        type=int,
        default=3,
    )

    parser.add_argument(
        "--depth-timeout",
        type=float,
        default=0.50,
    )

    parser.add_argument(
        "--dwell",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
    )

    parser.add_argument(
        "--key-step",
        type=float,
        default=0.0005,
    )

    parser.add_argument(
        "--j4-guard-deg",
        type=float,
        default=25.0,
    )

    parser.add_argument(
        "--j6-guard-deg",
        type=float,
        default=15.0,
    )

    parser.add_argument(
        "--high-follow",
        action="store_true",
    )

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    args = parser.parse_args()

    if args.hard_min_depth >= args.stop_depth:
        raise SystemExit(
            "--hard-min-depth必须小于--stop-depth"
        )

    if args.stop_depth >= args.slow_depth:
        raise SystemExit(
            "--stop-depth必须小于--slow-depth"
        )

    if (
        args.reach_margin < 0.0
        or args.reach_margin >= args.max_distance
    ):
        raise SystemExit(
            "--reach-margin必须大于等于0，"
            "并且小于--max-distance"
        )

    if not (
        0.50 <= args.depth_motion_scale <= 1.20
    ):
        raise SystemExit(
            "--depth-motion-scale必须在0.50～1.20之间"
        )

    if args.path_reserve < 0.0:
        raise SystemExit(
            "--path-reserve必须大于等于0"
        )

    rclpy.init()
    node = LeftArmDepthCycle()

    try:
        node.wait_driver()

        initial_depth = node.wait_for_depth()

        print("=" * 72)
        print("D435深度闭环参数")
        print("=" * 72)
        print(f"当前深度：{initial_depth:.3f} m")
        print(f"减速深度：{args.slow_depth:.3f} m")
        print(f"正常停止：{args.stop_depth:.3f} m")
        print(f"硬极限：{args.hard_min_depth:.3f} m")
        print(f"正常停留：{args.dwell:.1f} s")

        # stop_depth由启动参数定义，是探头完成检测位置时的相机标定读数。
        # 相机与探头之间的安装偏移已经包含在这个标定值中。
        raw_depth_delta = (
            initial_depth - args.stop_depth
        )

        # 相机深度变化与机械臂实际前伸不是严格1:1。
        # 本次实测：
        # 机械臂轨迹143mm，D435距离减少119mm，
        # 比例约0.832；默认保守使用0.80。
        required_travel = (
            raw_depth_delta
            / args.depth_motion_scale
        )

        executable_travel = (
            args.max_distance - args.reach_margin
        )

        absolute_max_camera_depth = (
            args.stop_depth
            + args.depth_motion_scale
            * args.max_distance
        )

        executable_max_camera_depth = (
            args.stop_depth
            + args.depth_motion_scale
            * executable_travel
        )

        print()
        print("=" * 72)
        print("任务可达性判断")
        print("=" * 72)
        print(
            f"相机当前识别距离："
            f"{initial_depth * 1000:.1f} mm"
        )
        print(
            f"探头按压标定距离："
            f"{args.stop_depth * 1000:.1f} mm"
        )
        print(
            f"需要减少的相机距离："
            f"{raw_depth_delta * 1000:.1f} mm"
        )
        print(
            f"深度/运动换算系数："
            f"{args.depth_motion_scale:.3f}"
        )
        print(
            f"换算后的机械臂所需前伸："
            f"{required_travel * 1000:.1f} mm"
        )
        print(
            f"当前配置的规划轨迹上限："
            f"{args.max_distance * 1000:.1f} mm"
        )
        print(
            f"量程安全余量："
            f"{args.reach_margin * 1000:.1f} mm"
        )
        print(
            f"当前任务最大储备轨迹："
            f"{executable_travel * 1000:.1f} mm"
        )
        print(
            f"按旧0.80换算估计的相机参考距离："
            f"{executable_max_camera_depth * 1000:.1f} mm"
        )
        print(
            f"按旧0.80换算估计的参考上限："
            f"{absolute_max_camera_depth * 1000:.1f} mm"
        )

        if initial_depth <= args.hard_min_depth:
            print()
            print("[TASK REJECTED] 当前目标距离过近")
            print(
                f"原因：相机距离{initial_depth:.3f} m，"
                f"已经达到或小于硬极限"
                f"{args.hard_min_depth:.3f} m。"
            )
            print(
                "继续前伸可能导致探头过压或机械碰撞，"
                "机械臂不执行任务。"
            )
            return

        if initial_depth <= args.stop_depth:
            print()
            print("[TASK REJECTED] 当前已经进入按压停止区")
            print(
                f"原因：相机距离{initial_depth:.3f} m，"
                f"已经达到或小于正常停止距离"
                f"{args.stop_depth:.3f} m。"
            )
            print(
                "当前不需要继续前伸，机械臂不执行任务。"
            )
            return

        # ========================================================
        # 长距离任务：旧0.80比例只作为估算，不再作为硬拒绝条件
        # ========================================================
        #
        # 以前：
        #
        #   required_travel =
        #       (initial_depth - stop_depth)
        #       / depth_motion_scale
        #
        # 然后只要required_travel超过设定量程就直接REJECT。
        #
        # 但Extended Reach已经允许：
        #   - 七轴构型重新分配
        #   - base -Y位置调整
        #   - adaptive arm angle
        #
        # 所以“相机距离变化 / base Z前伸”已经不再严格服从
        # 固定0.80比例。
        #
        # 当前自动规划已经验证500 mm轨迹可行；
        # 因此本次自动储备轨迹最多先使用500 mm。
        #
        # 最终是否真正可达，由D435实时闭环决定：
        #
        #   达到stop_depth
        #       -> 正常停止
        #       -> G300
        #
        #   储备轨迹走完仍未达到stop_depth
        #       -> 不执行G300
        #       -> 原路返回
        # ========================================================

        auto_plan_limit = min(
            executable_travel,
            0.500,
        )

        estimated_with_reserve = (
            required_travel
            + args.path_reserve
        )

        range_estimate_exceeded = (
            required_travel
            > auto_plan_limit
        )

        planned_distance = min(
            auto_plan_limit,
            estimated_with_reserve,
        )

        print()
        print("=" * 72)
        print("长距离规划策略")
        print("=" * 72)

        print(
            f"旧0.80模型估计需要前伸："
            f"{required_travel * 1000:.1f} mm"
        )

        print(
            f"轨迹预留请求："
            f"{args.path_reserve * 1000:.1f} mm"
        )

        print(
            f"旧模型估计+预留："
            f"{estimated_with_reserve * 1000:.1f} mm"
        )

        print(
            f"当前自动IK储备轨迹上限："
            f"{auto_plan_limit * 1000:.1f} mm"
        )

        if range_estimate_exceeded:
            print(
                "[RANGE MODEL WARN] "
                "旧0.80模型估计超过当前自动轨迹长度"
            )

        if estimated_with_reserve > auto_plan_limit:
            print(
                "[INFO] 本次不继续扩大IK轨迹，"
                "使用当前验证过的最大储备轨迹。"
            )

        print(
            "[INFO] 不再依据旧0.80模型直接拒绝任务。"
        )

        print(
            "[INFO] 是否真正到达检测位置由D435实时深度决定。"
        )

        print()
        print(
            "[TASK ACCEPTED] "
            "进入实际IK轨迹规划"
        )

        print(
            f"本次实际规划轨迹长度："
            f"{planned_distance * 1000:.1f} mm"
        )

        state0 = node.read_state()

        q0 = [
            float(value)
            for value in state0.joint[:7]
        ]

        controller_pose0 = [
            float(value)
            for value in state0.pose[:6]
        ]

        reason = singularity_reason(
            q0,
            args.j4_guard_deg,
            args.j6_guard_deg,
        )

        if reason:
            raise RuntimeError(
                f"当前姿态不能开始：{reason}"
            )

        rx_deg = math.degrees(controller_pose0[3])
        ry_deg = math.degrees(controller_pose0[4])

        if abs(rx_deg) > 0.5 or abs(ry_deg) > 0.5:
            raise RuntimeError(
                "探头没有保持水平："
                f"rx={rx_deg:.3f}°，"
                f"ry={ry_deg:.3f}°"
            )

        # 长距离任务启用RM75官方冗余遍历模式；
        # 短距离任务保持原来的单步模式。
        algo = create_algo(
            traversal_mode=(
                planned_distance > 0.350
            )
        )

        pose0 = list(
            algo.rm_algo_forward_kinematics(
                q0,
                1,
            )
        )

        # ========================================================
        # Extended Reach - 延迟下降
        #
        # planned_distance <= 350 mm:
        #   完全保持原有纯水平前伸，不改变Y。
        #
        # planned_distance > 350 mm:
        #   前340 mm仍完全水平；
        #   只有距离确实不够后才开始沿base -Y释放位置。
        #
        # 已验证：
        #   500 mm forward
        #   key_step = 0.5 mm
        #   drop_start = 340 mm
        #   final -Y = 200 mm
        # ========================================================

        pure_forward_limit = 0.350
        drop_start_m = 0.340
        maximum_drop_m = 0.200

        if planned_distance <= pure_forward_limit + 1e-9:
            reach_mode = "PURE_HORIZONTAL"
            extended_drop = 0.0

            key_progress, key_joints, key_phi = plan_path(
                algo=algo,
                q0=q0,
                pose0=pose0,
                distance_m=planned_distance,
                key_step_m=args.key_step,
                j4_guard_deg=args.j4_guard_deg,
                j6_guard_deg=args.j6_guard_deg,
                max_key_joint_step_deg=2.0,
            )

        else:
            reach_mode = "EXTENDED_DELAYED_DROP"

            # 340 mm之前始终为0。
            #
            # 500 mm时：
            #   (0.500 - 0.340) * 1.25
            #   = 0.200 m
            #
            # 中间距离按比例增加-Y，
            # 但最大不超过200 mm。
            extended_drop = min(
                maximum_drop_m,
                1.25 * max(
                    0.0,
                    planned_distance - drop_start_m,
                ),
            )

            key_progress, key_joints, key_phi = plan_path(
                algo=algo,
                q0=q0,
                pose0=pose0,
                distance_m=planned_distance,
                key_step_m=args.key_step,
                j4_guard_deg=min(
                    args.j4_guard_deg,
                    15.0,
                ),
                j6_guard_deg=args.j6_guard_deg,
                max_key_joint_step_deg=4.0,
                adaptive_phi=True,
                phi_search_deg=90.0,
                max_drop_m=extended_drop,
                drop_start_m=drop_start_m,
                drop_axis="y",
                drop_sign=-1.0,
            )

        print(
            f"[REACH MODE] {reach_mode}"
        )

        if reach_mode == "PURE_HORIZONTAL":
            print(
                "[REACH] 全程保持Y不变，纯水平前伸"
            )
        else:
            print(
                f"[REACH] 前{drop_start_m * 1000:.1f} mm"
                "保持Y不变"
            )
            print(
                f"[REACH] 之后才开始base -Y调整，"
                f"本轨迹最终-Y="
                f"{extended_drop * 1000:.1f} mm"
            )
            print(
                "[REACH] adaptive phi / traversal / search ±90°"
            )

        print()
        print("=" * 72)
        print("机械臂轨迹规划")
        print("=" * 72)
        print(
            f"本次规划前伸："
            f"{planned_distance * 1000:.1f} mm"
        )
        print(
            f"Extended Reach -Y偏移："
            f"{extended_drop * 1000:.1f} mm"
        )
        print(
            "Extended Reach臂角搜索：adaptive / ±90.0°"
        )
        print(
            f"臂角：{key_phi[0]:.3f}°"
            f" → {key_phi[-1]:.3f}°"
        )
        print(
            f"终点J4={key_joints[-1][3]:.3f}°，"
            f"J6={key_joints[-1][5]:.3f}°"
        )

        if not args.execute:
            print("[PLAN ONLY] 机械臂未运动")
            return

        traveled, stop_q, stop_reason, max_lateness = (
            node.approach(
                key_progress=key_progress,
                key_joints=key_joints,
                max_distance=planned_distance,
                stop_depth=args.stop_depth,
                hard_min_depth=args.hard_min_depth,
                slow_depth=args.slow_depth,
                fast_speed=args.fast_speed,
                slow_speed=args.slow_speed,
                acceleration=args.acceleration,
                deceleration=args.deceleration,
                confirm_frames=args.confirm_frames,
                depth_timeout=args.depth_timeout,
                rate=args.rate,
                high_follow=args.high_follow,
            )
        )

        print()
        print("=" * 72)
        print("前伸停止")
        print("=" * 72)
        print(f"停止原因：{stop_reason}")
        print(f"已前伸：{traveled * 1000:.1f} mm")

        normal_stop = stop_reason == "stop_depth"

        g300_ok = False
        g300_summary = {}

        if normal_stop:
            print(
                f"[G300 HOLD] 达到{args.stop_depth:.3f} m，"
                "保持当前位置并开始G300检测"
            )

            g300_ok, g300_summary = (
                run_g300_detection_while_holding(
                    node=node,
                    q_deg=stop_q,
                    rate=args.rate,
                    high_follow=args.high_follow,
                    hard_min_depth=args.hard_min_depth,
                )
            )

            if not g300_ok:
                print(
                    "[G300 ERROR] "
                    + str(
                        g300_summary.get(
                            "reason",
                            "未知G300错误",
                        )
                    ),
                    flush=True,
                )

        else:
            print(
                "[ABORT] 未按正常停止阈值结束，"
                "不执行G300检测，立即收回"
            )

            node.hold_joint(
                stop_q,
                duration=0.15,
                rate=args.rate,
                high_follow=args.high_follow,
            )

        node.return_to_start(
            key_progress=key_progress,
            key_joints=key_joints,
            traveled=traveled,
            max_distance=planned_distance,
            return_speed=args.return_speed,
            rate=args.rate,
            high_follow=args.high_follow,
        )

        time.sleep(0.5)

        final_state = node.read_state()

        final_pose = [
            float(value)
            for value in final_state.pose[:6]
        ]

        print()
        print("=" * 72)
        print("循环完成")
        print("=" * 72)
        print(
            f"Z返回残差："
            f"{(final_pose[2] - controller_pose0[2]) * 1000:.3f} mm"
        )
        print(
            f"X返回残差："
            f"{(final_pose[0] - controller_pose0[0]) * 1000:.3f} mm"
        )
        print(
            f"Y返回残差："
            f"{(final_pose[1] - controller_pose0[1]) * 1000:.3f} mm"
        )
        print(
            f"CANFD最大调度迟到："
            f"{max_lateness * 1000:.3f} ms"
        )
        print(
            f"机械臂状态：err={arm_error(final_state)}"
        )


        # ----------------------------------------------------
        # 保存机械臂 + G300统一任务结果
        # ----------------------------------------------------

        task_result_file = os.environ.get(
            "LEFT_PROBE_TASK_RESULT_FILE",
            "",
        ).strip()

        if task_result_file:

            task_payload = {
                "task_id":
                    os.environ.get(
                        "LEFT_PROBE_TASK_ID",
                        "",
                    ),

                "detection_type":
                    os.environ.get(
                        "LEFT_PROBE_DETECTION_TYPE",
                        "all",
                    ),

                "success":
                    bool(
                        normal_stop
                        and g300_ok
                    ),

                "stop_reason":
                    stop_reason,

                "traveled_m":
                    traveled,

                "arm_return": {
                    "x_residual_mm":
                        (
                            final_pose[0]
                            - controller_pose0[0]
                        )
                        * 1000.0,

                    "y_residual_mm":
                        (
                            final_pose[1]
                            - controller_pose0[1]
                        )
                        * 1000.0,

                    "z_residual_mm":
                        (
                            final_pose[2]
                            - controller_pose0[2]
                        )
                        * 1000.0,

                    "arm_error":
                        arm_error(
                            final_state
                        ),

                    "canfd_max_lateness_ms":
                        max_lateness
                        * 1000.0,
                },

                "g300":
                    g300_summary,
            }

            result_path = Path(
                task_result_file
            ).expanduser()

            result_path.parent.mkdir(
                parents=True,
                exist_ok=True,
            )

            result_path.write_text(
                json.dumps(
                    task_payload,
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )

            print(
                "[TASK RESULT FILE] "
                f"{result_path}"
            )

        if normal_stop and g300_ok:
            print(
                "[TASK SUCCESS] "
                "达到正常停止距离，"
                "G300检测完成并已返回初始位置"
            )

        elif normal_stop:
            print(
                "[TASK FAILED] "
                "达到正常停止距离，"
                "但G300检测失败；"
                "机械臂已安全返回初始位置"
            )

        else:
            print(
                "[TASK INCOMPLETE] "
                f"stop_reason={stop_reason}；"
                "未执行G300检测，"
                "机械臂已安全返回"
            )

    except KeyboardInterrupt:
        print("\n[STOP] 收到Ctrl+C，发送停止命令")
        node.stop()

    except Exception as exc:
        print(f"[ERROR] {exc}")
        node.stop()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
