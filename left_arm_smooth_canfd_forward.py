#!/usr/bin/env python3

import argparse
import bisect
import json
import math
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from rm_ros_interfaces.msg import (
    Armoriginalstate,
    Jointpos,
)

from Robotic_Arm.rm_robot_interface import (
    Algo,
    rm_force_type_e,
    rm_inverse_kinematics_params_t,
    rm_robot_arm_model_e,
)


NS = "/left_arm/rm_driver"

STATE_CMD = f"{NS}/get_current_arm_state_cmd"
STATE_RESULT = f"{NS}/get_current_arm_original_state_result"
CANFD_CMD = f"{NS}/movej_canfd_cmd"
STOP_CMD = f"{NS}/move_stop_cmd"

PLAN_FILE = Path.home() / "ros2_ws/left_arm_smooth_canfd_plan.json"

# 关节范围内再预留约2°
JOINT_LIMITS_DEG = [
    (-176.0, 176.0),   # J1
    (-128.0, 128.0),   # J2
    (-176.0, 176.0),   # J3
    (-133.0, 133.0),   # J4
    (-176.0, 176.0),   # J5
    (-126.0, 126.0),   # J6
    (-358.0, 358.0),   # J7
]

# 其余关节代价较高，J4/J6代价较低
JOINT_COST = [
    10.0,  # J1
    10.0,  # J2
    10.0,  # J3
    1.0,   # J4
    10.0,  # J5
    1.0,   # J6
    10.0,  # J7
]


def arm_error(msg: Armoriginalstate) -> int:
    value = msg.err

    if isinstance(value, (list, tuple)):
        return int(value[0]) if value else 0

    return int(value)


def unwrap_near(angle_deg: float, reference_deg: float) -> float:
    value = float(angle_deg)

    while value - reference_deg > 180.0:
        value -= 360.0

    while value - reference_deg < -180.0:
        value += 360.0

    return value


def smoothstep5(u: float) -> float:
    """
    五次S曲线：
    起点和终点的速度、加速度均为0。
    """
    u = max(0.0, min(1.0, u))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def singularity_reason(
    q_deg: List[float],
    j4_guard_deg: float,
    j6_guard_deg: float,
) -> Optional[str]:
    q2 = q_deg[1]
    q3 = q_deg[2]
    q4 = q_deg[3]
    q5 = q_deg[4]
    q6 = q_deg[5]

    if abs(q4) <= j4_guard_deg:
        return (
            f"J4={q4:.3f}°，进入"
            f"±{j4_guard_deg:.1f}°防奇异区"
        )

    if abs(q6) <= j6_guard_deg:
        return (
            f"J6={q6:.3f}°，进入"
            f"±{j6_guard_deg:.1f}°防奇异区"
        )

    if abs(q2) <= 5.0 and abs(q6) <= 8.0:
        return "J2、J6同时接近0°"

    if (
        abs(q2) <= 5.0
        and abs(abs(q3) - 90.0) <= 5.0
    ):
        return "J2接近0°且J3接近±90°"

    if (
        abs(q6) <= 8.0
        and abs(abs(q5) - 90.0) <= 5.0
    ):
        return "J6接近0°且J5接近±90°"

    return None


def joint_limit_reason(q_deg: List[float]) -> Optional[str]:
    for index, value in enumerate(q_deg):
        lower, upper = JOINT_LIMITS_DEG[index]

        if not lower <= value <= upper:
            return (
                f"J{index + 1}={value:.3f}°，"
                f"超出规划范围[{lower:.1f}, {upper:.1f}]°"
            )

    return None


class SmoothForward(Node):

    def __init__(self):
        super().__init__("left_arm_smooth_canfd_forward")

        self.state: Optional[Armoriginalstate] = None

        self.state_pub = self.create_publisher(
            Empty,
            STATE_CMD,
            10,
        )

        self.canfd_pub = self.create_publisher(
            Jointpos,
            CANFD_CMD,
            20,
        )

        self.stop_pub = self.create_publisher(
            Empty,
            STOP_CMD,
            10,
        )

        self.create_subscription(
            Armoriginalstate,
            STATE_RESULT,
            self.state_callback,
            10,
        )

    def state_callback(self, msg):
        self.state = msg

    def stop(self):
        self.stop_pub.publish(Empty())

    def wait_driver(self, timeout: float = 8.0):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            ready = (
                self.state_pub.get_subscription_count() >= 1
                and self.canfd_pub.get_subscription_count() >= 1
            )

            if ready:
                return

            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(
            "左臂状态或CANFD话题没有订阅者"
        )

    def read_state(self, timeout: float = 5.0):
        self.state = None
        self.state_pub.publish(Empty())

        deadline = time.monotonic() + timeout

        while self.state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.state is None:
            raise RuntimeError("未收到左臂状态")

        error = arm_error(self.state)

        if error != 0:
            raise RuntimeError(f"机械臂错误码：{error}")

        if len(self.state.joint) < 7:
            raise RuntimeError("关节状态数量不足")

        if len(self.state.pose) < 6:
            raise RuntimeError("末端位姿数量不足")

        return self.state

    def publish_joint(self, q_deg: List[float], high_follow: bool):
        msg = Jointpos()
        msg.joint = [
            math.radians(float(value))
            for value in q_deg
        ]
        msg.follow = bool(high_follow)
        msg.expand = 0.0
        msg.dof = 7

        self.canfd_pub.publish(msg)


def create_algo(
    traversal_mode: bool = False,
):
    algo = Algo(
        rm_robot_arm_model_e.RM_MODEL_RM_75_E,
        rm_force_type_e.RM_MODEL_RM_B_E,
    )

    # False:
    #   原来的单步模式，适合短距离连续Cartesian规划。
    #
    # True:
    #   官方冗余参数遍历模式。
    #   Extended Reach使用它扩大RM75七轴构型搜索。
    algo.rm_algo_set_redundant_parameter_traversal_mode(
        bool(traversal_mode)
    )

    return algo


def solve_keypoint(
    algo,
    q_previous: List[float],
    target_pose: List[float],
    phi_anchor: float,
    max_key_joint_step_deg: float,
    j4_guard_deg: float,
    j6_guard_deg: float,
    phi_search_deg: float = 2.0,
    traversal_fallback: bool = False,
) -> Tuple[List[float], float]:
    """
    对一个笛卡尔关键点搜索多个RM75臂角解。

    普通模式：
        phi_search_deg=2.0，与原实现接近。

    Extended Reach：
        以“上一关键点实际选中的臂角”为anchor，
        并允许更大的局部搜索范围。
        这样臂角可以沿路径逐渐累计漂移，
        而不是整条轨迹永久锁在初始phi0附近。
    """

    candidates = []

    maximum_search = max(
        0.0,
        float(phi_search_deg),
    )

    # 近处细搜索 + 远处稀疏搜索。
    # 比原来固定41次0.1°扫描更适合长轨迹，
    # 同时避免每个0.5mm关键点调用过多IK。
    magnitudes = (
        0.10,
        0.25,
        0.50,
        0.75,
        1.00,
        1.50,
        2.00,
        3.00,
        4.00,
        6.00,
        8.00,
        10.00,
        12.00,
        16.00,
        20.00,
        30.00,
        45.00,
    )

    offsets = [0.0]

    for magnitude in magnitudes:
        if magnitude <= maximum_search + 1e-9:
            offsets.extend(
                (
                    -magnitude,
                    +magnitude,
                )
            )

    for phi_offset in offsets:

        # RM75臂角规范到[-180, 180)
        phi = (
            (
                float(phi_anchor)
                + float(phi_offset)
                + 180.0
            )
            % 360.0
            - 180.0
        )

        params = rm_inverse_kinematics_params_t(
            q_previous,
            target_pose,
            1,
        )

        ret, solution = (
            algo.rm_algo_inverse_kinematics_rm75_for_arm_angle(
                params,
                phi,
            )
        )

        if ret != 0:
            continue

        q = [
            unwrap_near(
                solution[i],
                q_previous[i],
            )
            for i in range(7)
        ]

        dq = [
            q[i] - q_previous[i]
            for i in range(7)
        ]

        # 相邻关键点连续性仍然保持原来的强约束。
        if (
            max(abs(value) for value in dq)
            > max_key_joint_step_deg
        ):
            continue

        if joint_limit_reason(q) is not None:
            continue

        if (
            singularity_reason(
                q,
                j4_guard_deg,
                j6_guard_deg,
            )
            is not None
        ):
            continue

        # 保留原来的逐关节连续性代价。
        cost = sum(
            JOINT_COST[i] * dq[i] * dq[i]
            for i in range(7)
        )

        # 避免每一个0.5mm关键点无意义地大幅改变臂角。
        # 注意这里惩罚的是本关键点相对anchor的增量，
        # 不再惩罚相对整条轨迹最初phi0的累计变化。
        # 短距离仍然不鼓励无意义漂移；
        # 长行程时明显放松，让RM75充分利用冗余自由度。
        phi_weight = (
            0.20
            if maximum_search <= 2.0
            else 0.02
        )

        cost += (
            phi_weight
            * float(phi_offset)
            * float(phi_offset)
        )

        # 再轻微惩罚“某一个关节独自承担全部运动”，
        # 更倾向于7关节共同分摊。
        maximum_dq = max(
            abs(value)
            for value in dq
        )

        cost += (
            0.10
            * maximum_dq
            * maximum_dq
        )

        candidates.append(
            (
                cost,
                q,
                phi,
            )
        )

    if (
        not candidates
        and traversal_fallback
    ):
        # --------------------------------------------------------
        # Arm-angle IK没有找到候选时，
        # 不立即把目标判成“不可达”。
        #
        # 使用官方通用IK + traversal mode再搜索一次。
        # q_previous仍作为参考构型，因此不是无约束乱跳。
        # --------------------------------------------------------

        params = rm_inverse_kinematics_params_t(
            q_previous,
            target_pose,
            1,
        )

        ret, solution = (
            algo.rm_algo_inverse_kinematics(
                params
            )
        )

        if ret == 0:
            q = [
                unwrap_near(
                    solution[i],
                    q_previous[i],
                )
                for i in range(7)
            ]

            dq = [
                q[i] - q_previous[i]
                for i in range(7)
            ]

            # 仍然保留连续性硬门槛。
            if (
                max(abs(value) for value in dq)
                <= max_key_joint_step_deg
                and joint_limit_reason(q) is None
                and singularity_reason(
                    q,
                    j4_guard_deg,
                    j6_guard_deg,
                ) is None
            ):
                phi_ret, phi_solution = (
                    algo.rm_algo_calculate_arm_angle_from_config_rm75(
                        q
                    )
                )

                if phi_ret != 0:
                    phi_solution = phi_anchor

                fallback_cost = sum(
                    JOINT_COST[i]
                    * dq[i]
                    * dq[i]
                    for i in range(7)
                )

                candidates.append(
                    (
                        fallback_cost,
                        q,
                        float(phi_solution),
                    )
                )

    if not candidates:
        raise RuntimeError(
            "宽松IK搜索后仍无连续安全逆解；"
            f"phi_anchor={phi_anchor:.3f}°, "
            f"phi_search=±{maximum_search:.1f}°"
        )

    candidates.sort(
        key=lambda item: item[0]
    )

    _, q_best, phi_best = (
        candidates[0]
    )

    return q_best, phi_best


def plan_path(
    algo,
    q0: List[float],
    pose0: List[float],
    distance_m: float,
    key_step_m: float,
    j4_guard_deg: float,
    j6_guard_deg: float,
    max_key_joint_step_deg: float,
    adaptive_phi: bool = False,
    phi_search_deg: float = 2.0,
    max_drop_m: float = 0.0,
    drop_start_m: float = 0.0,
    drop_axis: str = "none",
    drop_sign: float = -1.0,
):
    """
    规划RM75连续关键点轨迹。

    普通模式：
        adaptive_phi=False
        max_drop_m=0
        行为与原规划器保持一致。

    Extended Reach模式：
        adaptive_phi=True
        每个关键点以phi_previous为中心搜索，
        允许臂角沿整条轨迹逐步漂移。

    可选下沉：
        max_drop_m > 0
        drop_axis = x / y
        drop_sign = -1 / +1

    base +Z仍然始终是主要水平前伸方向。
    末端姿态rx/ry/rz保持不变。
    """

    ret, phi0 = (
        algo.rm_algo_calculate_arm_angle_from_config_rm75(
            q0
        )
    )

    if ret != 0:
        raise RuntimeError(
            f"计算当前臂角失败：ret={ret}"
        )

    key_count = max(
        1,
        math.ceil(
            abs(distance_m)
            / key_step_m
        ),
    )

    key_progress = [0.0]
    key_joints = [list(q0)]
    key_phi = [float(phi0)]

    q_previous = list(q0)
    phi_previous = float(phi0)

    axis_index = {
        "none": None,
        "x": 0,
        "y": 1,
    }.get(str(drop_axis).lower())

    if axis_index is None:
        effective_drop = 0.0
    else:
        effective_drop = max(
            0.0,
            float(max_drop_m),
        )

    direction = (
        -1.0
        if float(drop_sign) < 0.0
        else +1.0
    )

    for index in range(
        1,
        key_count + 1,
    ):
        progress = (
            index / key_count
        )

        target_pose = list(pose0)

        # --------------------------------------------------------
        # 主运动：base +Z水平前进
        # --------------------------------------------------------

        target_pose[2] = (
            pose0[2]
            + distance_m * progress
        )

        # --------------------------------------------------------
        # 可选：允许末端随前伸逐渐下沉
        #
        # smoothstep：
        # 起点、终点斜率均为0，
        # 不会突然改变Cartesian方向。
        # --------------------------------------------------------

        if (
            axis_index is not None
            and effective_drop > 0.0
        ):
            # ----------------------------------------------------
            # 延迟下降：
            #
            # 在 drop_start_m 之前完全保持原来的Y位置，
            # 只沿base +Z水平前伸。
            #
            # 只有超过drop_start_m以后，
            # 才开始用smoothstep逐渐释放-Y自由度。
            # ----------------------------------------------------

            forward_now = (
                abs(distance_m)
                * progress
            )

            drop_start = min(
                max(
                    0.0,
                    float(drop_start_m),
                ),
                abs(distance_m),
            )

            if (
                forward_now
                <= drop_start
            ):
                drop_progress = 0.0

            elif (
                abs(distance_m)
                <= drop_start + 1e-9
            ):
                drop_progress = 0.0

            else:
                drop_progress = (
                    (
                        forward_now
                        - drop_start
                    )
                    /
                    (
                        abs(distance_m)
                        - drop_start
                    )
                )

                drop_progress = min(
                    1.0,
                    max(
                        0.0,
                        drop_progress,
                    ),
                )

            smooth = (
                drop_progress
                * drop_progress
                * (
                    3.0
                    - 2.0
                    * drop_progress
                )
            )

            target_pose[axis_index] = (
                pose0[axis_index]
                + direction
                * effective_drop
                * smooth
            )

        if adaptive_phi:
            phi_anchor = (
                phi_previous
            )

            local_search = max(
                2.0,
                float(phi_search_deg),
            )

        else:
            # 完全保留旧行为：
            # 每一点始终围绕最初phi0。
            phi_anchor = float(phi0)
            local_search = 2.0

        q_target, phi_target = (
            solve_keypoint(
                algo=algo,
                q_previous=q_previous,
                target_pose=target_pose,
                phi_anchor=phi_anchor,
                max_key_joint_step_deg=(
                    max_key_joint_step_deg
                ),
                j4_guard_deg=j4_guard_deg,
                j6_guard_deg=j6_guard_deg,
                phi_search_deg=local_search,
                traversal_fallback=adaptive_phi,
            )
        )

        key_progress.append(
            progress
        )

        key_joints.append(
            q_target
        )

        key_phi.append(
            phi_target
        )

        q_previous = q_target
        phi_previous = phi_target

    return (
        key_progress,
        key_joints,
        key_phi,
    )


def interpolate_path(
    progress: float,
    key_progress: List[float],
    key_joints: List[List[float]],
) -> List[float]:
    if progress <= 0.0:
        return list(key_joints[0])

    if progress >= 1.0:
        return list(key_joints[-1])

    right = bisect.bisect_right(
        key_progress,
        progress,
    )

    left = right - 1

    p0 = key_progress[left]
    p1 = key_progress[right]

    ratio = (progress - p0) / (p1 - p0)

    return [
        key_joints[left][i]
        + ratio
        * (
            key_joints[right][i]
            - key_joints[left][i]
        )
        for i in range(7)
    ]


def wait_until(target_time: float):
    while True:
        remaining = target_time - time.perf_counter()

        if remaining <= 0.0:
            return

        if remaining > 0.002:
            time.sleep(remaining - 0.001)
        else:
            # 最后约1 ms短暂等待，减小周期抖动
            pass


def stream_path(
    node: SmoothForward,
    key_progress: List[float],
    key_joints: List[List[float]],
    duration_s: float,
    rate_hz: float,
    high_follow: bool,
    reverse: bool = False,
):
    period = 1.0 / rate_hz
    samples = max(20, int(round(duration_s * rate_hz)))

    # 先连续发送当前起始点
    start_q = (
        key_joints[-1]
        if reverse
        else key_joints[0]
    )

    prime_count = max(10, int(0.2 * rate_hz))

    prime_start = time.perf_counter()

    for index in range(prime_count):
        node.publish_joint(start_q, high_follow)
        rclpy.spin_once(node, timeout_sec=0.0)

        wait_until(
            prime_start + (index + 1) * period
        )

    start_time = time.perf_counter()
    maximum_lateness = 0.0

    for index in range(samples + 1):
        u = index / samples
        profile = smoothstep5(u)

        if reverse:
            progress = 1.0 - profile
        else:
            progress = profile

        q = interpolate_path(
            progress,
            key_progress,
            key_joints,
        )

        deadline = start_time + index * period
        wait_until(deadline)

        lateness = time.perf_counter() - deadline
        maximum_lateness = max(
            maximum_lateness,
            lateness,
        )

        node.publish_joint(q, high_follow)
        rclpy.spin_once(node, timeout_sec=0.0)

    # 末端保持0.3 s
    final_q = (
        key_joints[0]
        if reverse
        else key_joints[-1]
    )

    hold_count = max(10, int(0.3 * rate_hz))
    hold_start = time.perf_counter()

    for index in range(hold_count):
        node.publish_joint(final_q, high_follow)
        rclpy.spin_once(node, timeout_sec=0.0)

        wait_until(
            hold_start + (index + 1) * period
        )

    return maximum_lateness


def print_plan(
    q0,
    pose0,
    key_joints,
    key_phi,
    distance_m,
    duration_s,
):
    q_final = key_joints[-1]

    delta = [
        q_final[i] - q0[i]
        for i in range(7)
    ]

    average_speed = abs(distance_m) / duration_s
    peak_speed = 1.875 * average_speed

    print()
    print("=" * 76)
    print("长距离平滑轨迹规划完成")
    print("=" * 76)

    print(
        f"起点：x={pose0[0]:.6f}, "
        f"y={pose0[1]:.6f}, "
        f"z={pose0[2]:.6f} m"
    )

    print(
        f"目标Z：{pose0[2] + distance_m:.6f} m"
    )

    print(
        f"水平距离：{distance_m * 1000:.1f} mm"
    )

    print(f"运动时间：{duration_s:.3f} s")

    print(
        f"平均速度：{average_speed * 1000:.1f} mm/s"
    )

    print(
        f"S曲线峰值速度约："
        f"{peak_speed * 1000:.1f} mm/s"
    )

    print(f"臂角：{key_phi[0]:.3f}° → {key_phi[-1]:.3f}°")

    print()
    print("最终关节与总变化：")

    for index in range(7):
        marker = "  ← 优先" if index in (3, 5) else ""

        print(
            f"J{index + 1}: "
            f"{q0[index]:9.3f}°"
            f" → {q_final[index]:9.3f}°"
            f"  Δ={delta[index]:+8.3f}°"
            f"{marker}"
        )

    print()
    print(
        f"最终J4裕量：|{q_final[3]:.3f}°|"
    )
    print(
        f"最终J6裕量：|{q_final[5]:.3f}°|"
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--distance",
        type=float,
        default=0.060,
        help="base Z方向距离，单位m",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=2.5,
        help="单程运动时间，单位s",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
        help="CANFD发送频率，默认100Hz",
    )

    parser.add_argument(
        "--key-step",
        type=float,
        default=0.0005,
        help="逆解关键点间距，默认0.5mm",
    )

    parser.add_argument(
        "--j4-guard-deg",
        type=float,
        default=20.0,
    )

    parser.add_argument(
        "--j6-guard-deg",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--max-key-joint-step-deg",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--high-follow",
        action="store_true",
        help="启用高跟随；默认使用低跟随",
    )

    parser.add_argument(
        "--round-trip",
        action="store_true",
        help="前伸后等待1秒，再沿同一路径收回",
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="实际执行；未指定时只规划",
    )


    parser.add_argument(
        "--extended-reach",
        action="store_true",
        help=(
            "启用长行程模式："
            "臂角以每个上一关键点为中心累计漂移"
        ),
    )

    parser.add_argument(
        "--phi-search-deg",
        type=float,
        default=8.0,
        help=(
            "Extended Reach每个关键点的"
            "局部臂角搜索范围"
        ),
    )

    parser.add_argument(
        "--max-drop",
        type=float,
        default=0.0,
        help=(
            "末端允许累计下沉/偏移的最大距离(m)"
        ),
    )

    parser.add_argument(
        "--drop-start",
        type=float,
        default=0.0,
        help=(
            "前伸多少米后才开始允许Cartesian偏移；"
            "之前保持完全水平前伸"
        ),
    )

    parser.add_argument(
        "--drop-axis",
        choices=(
            "none",
            "x",
            "y",
        ),
        default="none",
        help=(
            "允许偏移的base轴；"
            "base Z保留为前伸方向"
        ),
    )

    parser.add_argument(
        "--drop-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=-1.0,
        help=(
            "允许偏移方向：-1或+1"
        ),
    )

    parser.add_argument(
        "--allow-current-orientation",
        action="store_true",
        help=(
            "允许从当前已经实机验证的非水平末端姿态继续规划；"
            "仅用于长行程锚点测试"
        ),
    )

    args = parser.parse_args()

    if abs(args.distance) < 1e-6:
        raise SystemExit("--distance不能为0")

    if args.duration <= 0.0:
        raise SystemExit("--duration必须大于0")

    if args.rate < 50.0:
        raise SystemExit("--rate建议不低于50Hz")

    rclpy.init()
    node = SmoothForward()

    try:
        node.wait_driver()

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
                f"当前姿态不允许开始：{reason}"
            )

        # 当前正式检测姿态应接近rx=0、ry=0
        if (
            not args.allow_current_orientation
            and (
                abs(math.degrees(controller_pose0[3])) > 0.5
                or abs(math.degrees(controller_pose0[4])) > 0.5
            )
        ):
            raise RuntimeError(
                "当前探头尚未水平："
                f"rx={math.degrees(controller_pose0[3]):.3f}°，"
                f"ry={math.degrees(controller_pose0[4]):.3f}°；"
                "已知可达锚点测试可使用 "
                "--allow-current-orientation"
            )

        algo = create_algo(
            traversal_mode=(
                args.extended_reach
            )
        )

        pose0 = list(
            algo.rm_algo_forward_kinematics(
                q0,
                1,
            )
        )

        key_progress, key_joints, key_phi = plan_path(
            algo=algo,
            q0=q0,
            pose0=pose0,
            distance_m=args.distance,
            key_step_m=args.key_step,
            j4_guard_deg=args.j4_guard_deg,
            j6_guard_deg=args.j6_guard_deg,
            max_key_joint_step_deg=(
                args.max_key_joint_step_deg
            ),
            adaptive_phi=(
                args.extended_reach
            ),
            phi_search_deg=(
                args.phi_search_deg
            ),
            max_drop_m=(
                args.max_drop
                if args.extended_reach
                else 0.0
            ),
            drop_start_m=(
                args.drop_start
                if args.extended_reach
                else 0.0
            ),
            drop_axis=(
                args.drop_axis
                if args.extended_reach
                else "none"
            ),
            drop_sign=args.drop_sign,
        )

        print_plan(
            q0=q0,
            pose0=pose0,
            key_joints=key_joints,
            key_phi=key_phi,
            distance_m=args.distance,
            duration_s=args.duration,
        )

        plan_payload = {
            "distance_m": args.distance,
            "duration_s": args.duration,
            "rate_hz": args.rate,
            "high_follow": args.high_follow,
            "round_trip": args.round_trip,
            "extended_reach": args.extended_reach,
            "phi_search_deg": args.phi_search_deg,
            "max_drop_m": (
                args.max_drop
                if args.extended_reach
                else 0.0
            ),
            "drop_start_m": (
                args.drop_start
                if args.extended_reach
                else 0.0
            ),
            "drop_axis": (
                args.drop_axis
                if args.extended_reach
                else "none"
            ),
            "drop_sign": args.drop_sign,
            "start_joint_deg": q0,
            "final_joint_deg": key_joints[-1],
            "start_pose": pose0,
            "arm_angle_anchor_deg": key_phi[0],
            "arm_angle_start_deg": key_phi[0],
            "arm_angle_final_deg": key_phi[-1],
            "arm_angle_drift_deg": key_phi[-1] - key_phi[0],
            "key_point_count": len(key_joints),
            "key_progress": key_progress,
            "key_joints_deg": key_joints,
            "key_arm_angles_deg": key_phi,
        }

        PLAN_FILE.write_text(
            json.dumps(
                plan_payload,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

        print(f"规划摘要已保存：{PLAN_FILE}")

        if not args.execute:
            print()
            print("[PLAN ONLY] 机械臂未运动")
            print("确认最终J4/J6及目标距离后，加 --execute")
            return

        print()
        print("=" * 76)
        print(
            "开始CANFD连续透传："
            f"{'高跟随' if args.high_follow else '低跟随'}"
        )
        print("=" * 76)

        max_lateness = stream_path(
            node=node,
            key_progress=key_progress,
            key_joints=key_joints,
            duration_s=args.duration,
            rate_hz=args.rate,
            high_follow=args.high_follow,
            reverse=False,
        )

        if args.round_trip:
            print("[hold] 前端保持1秒")
            time.sleep(1.0)

            print("[return] 沿相同关节轨迹平滑收回")

            max_lateness = max(
                max_lateness,
                stream_path(
                    node=node,
                    key_progress=key_progress,
                    key_joints=key_joints,
                    duration_s=args.duration,
                    rate_hz=args.rate,
                    high_follow=args.high_follow,
                    reverse=True,
                ),
            )

        time.sleep(0.5)

        final_state = node.read_state()

        final_pose = [
            float(value)
            for value in final_state.pose[:6]
        ]

        final_q = [
            float(value)
            for value in final_state.joint[:7]
        ]

        expected_distance = (
            0.0
            if args.round_trip
            else args.distance
        )

        actual_distance = (
            final_pose[2] - controller_pose0[2]
        )

        print()
        print("=" * 76)
        print("执行结果")
        print("=" * 76)

        print(
            f"期望base Z变化："
            f"{expected_distance * 1000:.3f} mm"
        )

        print(
            f"实际base Z变化："
            f"{actual_distance * 1000:.3f} mm"
        )

        print(
            f"X漂移："
            f"{(final_pose[0] - controller_pose0[0]) * 1000:.3f} mm"
        )

        print(
            f"Y漂移："
            f"{(final_pose[1] - controller_pose0[1]) * 1000:.3f} mm"
        )

        print(
            f"最终姿态："
            f"rx={math.degrees(final_pose[3]):.3f}°，"
            f"ry={math.degrees(final_pose[4]):.3f}°，"
            f"rz={math.degrees(final_pose[5]):.3f}°"
        )

        print(
            f"最终J4={final_q[3]:.3f}°，"
            f"J6={final_q[5]:.3f}°"
        )

        print(
            f"CANFD最大调度迟到："
            f"{max_lateness * 1000:.3f} ms"
        )

        print(f"机械臂状态：err={arm_error(final_state)}")

    except KeyboardInterrupt:
        print("\n[stop] 收到Ctrl+C，发送停止命令")
        node.stop()

    except Exception as exc:
        print(f"[error] {exc}")
        node.stop()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
