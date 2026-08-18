#!/usr/bin/env python3

import argparse
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class D435Distance(Node):

    def __init__(self, args):
        super().__init__("left_d435_distance")

        self.cx = args.center_x
        self.cy = args.center_y
        self.roi_w = args.roi_width
        self.roi_h = args.roi_height
        self.scale = args.scale
        self.min_valid_ratio = args.min_valid_ratio
        self.print_period = 1.0 / args.print_rate

        self.median_history = deque(maxlen=args.filter_frames)
        self.near_history = deque(maxlen=args.filter_frames)

        self.last_print = 0.0
        self.reported_format = False

        # D435近距离盲区处理
        self.too_close_threshold = args.too_close_threshold
        self.too_close_confirm_frames = (
            args.too_close_confirm_frames
        )
        self.invalid_streak = 0
        self.last_valid_near = None
        self.last_valid_time = 0.0
        self.too_close_latched = False

        self.median_pub = self.create_publisher(
            Float32,
            "/left_probe/distance_m",
            10,
        )
        self.near_pub = self.create_publisher(
            Float32,
            "/left_probe/near_distance_m",
            10,
        )
        self.valid_pub = self.create_publisher(
            Bool,
            "/left_probe/distance_valid",
            10,
        )

        self.too_close_pub = self.create_publisher(
            Bool,
            "/left_probe/too_close",
            10,
        )

        self.create_subscription(
            Image,
            args.topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"depth topic: {args.topic}")
        self.get_logger().info(
            f"ROI center=({self.cx},{self.cy}), "
            f"size={self.roi_w}x{self.roi_h}"
        )

    def publish_invalid(self, reason):
        now = time.monotonic()

        valid_msg = Bool()
        valid_msg.data = False
        self.valid_pub.publish(valid_msg)

        self.invalid_streak += 1

        recently_valid = (
            self.last_valid_near is not None
            and now - self.last_valid_time <= 0.5
        )

        close_before_loss = (
            recently_valid
            and self.last_valid_near
            <= self.too_close_threshold
        )

        if (
            close_before_loss
            and self.invalid_streak
            >= self.too_close_confirm_frames
        ):
            self.too_close_latched = True

        close_msg = Bool()
        close_msg.data = self.too_close_latched
        self.too_close_pub.publish(close_msg)

        if now - self.last_print >= self.print_period:
            if self.too_close_latched:
                print(
                    "[TOO_CLOSE] "
                    f"last_near={self.last_valid_near:.3f} m  "
                    f"invalid_frames={self.invalid_streak}  "
                    f"{reason}",
                    flush=True,
                )
            else:
                print(
                    f"[INVALID] {reason}",
                    flush=True,
                )

            self.last_print = now

    def decode_depth(self, msg):
        encoding = msg.encoding.upper()

        if encoding in ("16UC1", "MONO16"):
            dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
            columns = msg.step // 2

            image = np.frombuffer(
                msg.data,
                dtype=dtype,
            ).reshape(msg.height, columns)

            return (
                image[:, :msg.width].astype(np.float32)
                * self.scale
            )

        if encoding == "32FC1":
            dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
            columns = msg.step // 4

            image = np.frombuffer(
                msg.data,
                dtype=dtype,
            ).reshape(msg.height, columns)

            return image[:, :msg.width].astype(np.float32)

        raise ValueError(f"unsupported encoding: {msg.encoding}")

    def depth_callback(self, msg):
        if not self.reported_format:
            print(
                f"depth={msg.width}x{msg.height}, "
                f"encoding={msg.encoding}, step={msg.step}",
                flush=True,
            )
            self.reported_format = True

        try:
            depth = self.decode_depth(msg)
        except Exception as exc:
            self.publish_invalid(str(exc))
            return

        height, width = depth.shape

        cx = self.cx if self.cx >= 0 else width // 2
        cy = self.cy if self.cy >= 0 else height // 2

        x0 = max(0, cx - self.roi_w // 2)
        x1 = min(width, cx + self.roi_w // 2)
        y0 = max(0, cy - self.roi_h // 2)
        y1 = min(height, cy + self.roi_h // 2)

        roi = depth[y0:y1, x0:x1]

        valid_mask = (
            np.isfinite(roi)
            & (roi > 0.05)
            & (roi < 8.0)
        )

        values = roi[valid_mask]
        valid_ratio = values.size / max(1, roi.size)

        if (
            values.size == 0
            or valid_ratio < self.min_valid_ratio
        ):
            self.median_history.clear()
            self.near_history.clear()
            self.publish_invalid(
                f"valid={valid_ratio * 100:.1f}%"
            )
            return

        median = float(np.median(values))
        near = float(np.percentile(values, 20.0))

        self.median_history.append(median)
        self.near_history.append(near)

        median_filtered = float(np.median(self.median_history))
        near_filtered = float(np.median(self.near_history))

        # 恢复有效深度后解除too-close锁存
        self.invalid_streak = 0
        self.last_valid_near = near_filtered
        self.last_valid_time = time.monotonic()
        self.too_close_latched = False

        close_msg = Bool()
        close_msg.data = False
        self.too_close_pub.publish(close_msg)

        valid_msg = Bool()
        valid_msg.data = True
        self.valid_pub.publish(valid_msg)

        median_msg = Float32()
        median_msg.data = median_filtered
        self.median_pub.publish(median_msg)

        near_msg = Float32()
        near_msg.data = near_filtered
        self.near_pub.publish(near_msg)

        now = time.monotonic()

        if now - self.last_print >= self.print_period:
            print(
                f"median={median_filtered:.3f} m  "
                f"near={near_filtered:.3f} m  "
                f"valid={valid_ratio * 100:5.1f}%  "
                f"ROI=({x0}:{x1},{y0}:{y1})",
                flush=True,
            )
            self.last_print = now


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--topic",
        default="/left_probe/d435/depth/image_rect_raw",
    )
    parser.add_argument("--center-x", type=int, default=-1)
    parser.add_argument("--center-y", type=int, default=-1)
    parser.add_argument("--roi-width", type=int, default=60)
    parser.add_argument("--roi-height", type=int, default=60)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument(
        "--min-valid-ratio",
        type=float,
        default=0.60,
    )
    parser.add_argument(
        "--filter-frames",
        type=int,
        default=7,
    )

    parser.add_argument(
        "--too-close-threshold",
        type=float,
        default=0.23,
        help=(
            "最后有效near距离小于该值后发生深度丢失，"
            "判定为过近，默认0.23m"
        ),
    )

    parser.add_argument(
        "--too-close-confirm-frames",
        type=int,
        default=3,
        help="连续无效帧确认数，默认3帧",
    )
    parser.add_argument(
        "--print-rate",
        type=float,
        default=5.0,
    )

    args = parser.parse_args()

    rclpy.init()
    node = D435Distance(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n退出测距")
    finally:
        node.destroy_node()

        # Ctrl+C时ROS上下文可能已自动shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
