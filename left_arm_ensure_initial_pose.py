#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty
from rm_ros_interfaces.msg import Armoriginalstate, Movej


NS = "/left_arm/rm_driver"

STATE_CMD = f"{NS}/get_current_arm_state_cmd"
STATE_RESULT = f"{NS}/get_current_arm_original_state_result"

MOVEJ_CMD = f"{NS}/movej_cmd"
MOVEJ_RESULT = f"{NS}/movej_result"


def extract_joint_deg(data: Dict[str, Any]) -> List[float]:
    """兼容不同初始位姿JSON结构。"""

    for key in (
        "joint_deg",
        "joints_deg",
        "joint",
        "joints",
    ):
        value = data.get(key)

        if isinstance(value, list) and len(value) >= 7:
            return [
                float(item)
                for item in value[:7]
            ]

        if isinstance(value, dict):
            result = []

            for index in range(1, 8):
                found = None

                for candidate in (
                    f"J{index}",
                    f"j{index}",
                    str(index),
                    str(index - 1),
                ):
                    if candidate in value:
                        found = float(value[candidate])
                        break

                if found is None:
                    result = []
                    break

                result.append(found)

            if len(result) == 7:
                return result

    for key in (
        "state",
        "arm_state",
        "initial_state",
        "current_state",
    ):
        value = data.get(key)

        if isinstance(value, dict):
            try:
                return extract_joint_deg(value)
            except ValueError:
                pass

    raise ValueError(
        "JSON中没有找到7轴关节角："
        "需要joint_deg、joints_deg、joint或joints字段"
    )


def arm_error(msg: Armoriginalstate) -> int:
    value = msg.err

    if isinstance(value, (list, tuple)):
        return int(value[0]) if value else 0

    return int(value)


def angular_delta_deg(
    current: float,
    target: float,
    allow_wrap: bool,
) -> float:
    delta = target - current

    if allow_wrap:
        while delta > 180.0:
            delta -= 360.0

        while delta < -180.0:
            delta += 360.0

    return delta


def calculate_deltas(
    current: List[float],
    target: List[float],
) -> List[float]:
    return [
        angular_delta_deg(
            current=current[index],
            target=target[index],
            allow_wrap=(index == 6),
        )
        for index in range(7)
    ]


def target_near_current(
    current: List[float],
    saved_target: List[float],
) -> List[float]:
    """
    J1～J6直接采用保存角度。
    J7选择与当前角度最近的等价角，防止跨360°旋转。
    """

    target = list(saved_target)

    j7_delta = angular_delta_deg(
        current=current[6],
        target=saved_target[6],
        allow_wrap=True,
    )

    target[6] = current[6] + j7_delta

    return target


class InitialPoseManager(Node):

    def __init__(self):
        super().__init__(
            "left_arm_ensure_initial_pose"
        )

        self.state: Optional[Armoriginalstate] = None
        self.move_result: Optional[bool] = None

        self.state_pub = self.create_publisher(
            Empty,
            STATE_CMD,
            10,
        )

        self.movej_pub = self.create_publisher(
            Movej,
            MOVEJ_CMD,
            10,
        )

        self.create_subscription(
            Armoriginalstate,
            STATE_RESULT,
            self.state_callback,
            10,
        )

        self.create_subscription(
            Bool,
            MOVEJ_RESULT,
            self.move_result_callback,
            10,
        )

    def state_callback(self, msg):
        self.state = msg

    def move_result_callback(self, msg):
        self.move_result = bool(msg.data)

    def wait_driver(self, timeout: float = 10.0):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            state_ready = (
                self.state_pub.get_subscription_count()
                >= 1
            )

            move_ready = (
                self.movej_pub.get_subscription_count()
                >= 1
            )

            if state_ready and move_ready:
                print(
                    "[PASS] 左臂驱动接口已就绪"
                )
                return

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        raise RuntimeError(
            "左臂状态或MoveJ接口未就绪"
        )

    def read_state(
        self,
        timeout: float = 5.0,
    ) -> Armoriginalstate:
        self.state = None
        self.state_pub.publish(Empty())

        deadline = time.monotonic() + timeout

        while (
            self.state is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        if self.state is None:
            raise RuntimeError(
                "读取左臂状态超时"
            )

        error = arm_error(self.state)

        if error != 0:
            raise RuntimeError(
                f"机械臂状态异常：err={error}"
            )

        if len(self.state.joint) < 7:
            raise RuntimeError(
                "机械臂返回的关节数量不足7个"
            )

        return self.state

    def movej(
        self,
        target_deg: List[float],
        speed: int,
        timeout: float,
    ):
        self.move_result = None

        msg = Movej()

        msg.joint = [
            math.radians(value)
            for value in target_deg
        ]

        msg.speed = int(speed)
        msg.block = True

        if hasattr(msg, "trajectory_connect"):
            msg.trajectory_connect = 0

        if hasattr(msg, "dof"):
            msg.dof = 7

        print()
        print("=" * 72)
        print("直接执行初始位置复位")
        print("=" * 72)
        print(f"MoveJ速度：{speed}")

        for index, value in enumerate(
            target_deg,
            start=1,
        ):
            print(
                f"J{index}={value:.3f}°",
                end="\n" if index == 7 else "  ",
            )

        self.movej_pub.publish(msg)

        deadline = time.monotonic() + timeout

        while (
            self.move_result is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        if self.move_result is None:
            raise RuntimeError(
                f"等待MoveJ结果超时：{timeout:.1f}s"
            )

        if not self.move_result:
            raise RuntimeError(
                "MoveJ执行失败"
            )

        print("[PASS] MoveJ执行完成")


def print_comparison(
    current: List[float],
    target: List[float],
):
    deltas = calculate_deltas(
        current,
        target,
    )

    print()
    print("=" * 72)
    print("初始位置检查")
    print("=" * 72)

    for index in range(7):
        print(
            f"J{index + 1}: "
            f"当前={current[index]:9.3f}°  "
            f"初始={target[index]:9.3f}°  "
            f"差值={deltas[index]:+8.3f}°"
        )

    maximum = max(
        abs(value)
        for value in deltas
    )

    print(
        f"最大关节偏差：{maximum:.3f}°"
    )

    return maximum


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pose-file",
        required=True,
    )

    parser.add_argument(
        "--move",
        action="store_true",
        help="不在初始位置时直接执行复位",
    )

    parser.add_argument(
        "--joint-tolerance-deg",
        type=float,
        default=0.50,
    )

    parser.add_argument(
        "--verify-tolerance-deg",
        type=float,
        default=0.60,
    )

    parser.add_argument(
        "--speed",
        type=int,
        default=5,
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
    )

    args = parser.parse_args()

    pose_path = Path(
        args.pose_file
    ).expanduser()

    if not pose_path.exists():
        raise SystemExit(
            f"初始位置文件不存在：{pose_path}"
        )

    try:
        data = json.loads(
            pose_path.read_text(
                encoding="utf-8"
            )
        )

        saved_joint_deg = extract_joint_deg(
            data
        )

    except Exception as exc:
        raise SystemExit(
            f"读取初始位置文件失败：{exc}"
        )

    print(f"初始位置文件：{pose_path}")

    rclpy.init()
    node = InitialPoseManager()

    try:
        node.wait_driver()

        state0 = node.read_state()

        current_joint = [
            float(value)
            for value in state0.joint[:7]
        ]

        maximum_delta = print_comparison(
            current_joint,
            saved_joint_deg,
        )

        if (
            maximum_delta
            <= args.joint_tolerance_deg
        ):
            print()
            print(
                "[INITIAL READY] "
                "机械臂已经处于初始位置"
            )
            return 0

        print()
        print(
            "[NOT INITIAL] "
            "机械臂不在初始位置"
        )

        if not args.move:
            print(
                "[CHECK ONLY] 未发送复位指令"
            )
            return 3

        command_target = target_near_current(
            current_joint,
            saved_joint_deg,
        )

        node.movej(
            target_deg=command_target,
            speed=args.speed,
            timeout=args.timeout,
        )

        time.sleep(0.8)

        state1 = node.read_state()

        actual_joint = [
            float(value)
            for value in state1.joint[:7]
        ]

        verify_error = print_comparison(
            actual_joint,
            saved_joint_deg,
        )

        if (
            verify_error
            > args.verify_tolerance_deg
        ):
            raise RuntimeError(
                "复位后校验失败："
                f"最大关节误差"
                f"{verify_error:.3f}°，"
                f"允许值"
                f"{args.verify_tolerance_deg:.3f}°"
            )

        print()
        print(
            "[INITIAL RESTORED] "
            "机械臂已直接复位到初始位置"
        )

        return 0

    except Exception as exc:
        print(f"[ERROR] {exc}")
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
