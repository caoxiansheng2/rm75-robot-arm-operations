#!/usr/bin/env python3

import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import serial


# ============================================================
# G300 正式 v2.72 四种带放电分析采集方式
# ============================================================

CHANNELS = [
    {
        "name": "open_ultrasonic",
        "name_cn": "开放式超声波",
        "code": 0x01,
        "group": "ultrasonic",
    },
    {
        "name": "tev",
        "name_cn": "地电波 TEV",
        "code": 0x11,
        "group": "high_frequency",
    },
    {
        "name": "contact_ultrasonic",
        "name_cn": "接触式超声波",
        "code": 0x21,
        "group": "ultrasonic",
    },
    {
        "name": "uhf",
        "name_cn": "特高频 UHF",
        "code": 0x31,
        "group": "high_frequency",
    },
]


DISCHARGE_TYPE = {
    0: "无放电",
    1: "内部放电",
    2: "表面放电",
    3: "悬浮电位",
    4: "电晕放电",
    9: "分析未完成",
}


INTENSITY = {
    0: "非放电",
    1: "低水平放电",
    2: "中水平放电",
    3: "高水平放电",
    9: "分析未完成",
}


SYNC_NAME = {
    0: "内同步",
    1: "外同步",
}


def now_iso():
    return datetime.now().astimezone().isoformat(
        timespec="milliseconds"
    )


# ============================================================
# Modbus CRC
# ============================================================

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF

    for b in data:
        crc ^= b

        for _ in range(8):
            if crc & 1:
                crc = (
                    (crc >> 1)
                    ^ 0xA001
                )
            else:
                crc >>= 1

    return crc & 0xFFFF


def add_crc(payload: bytes) -> bytes:
    crc = crc16_modbus(payload)

    return payload + bytes([
        crc & 0xFF,
        (crc >> 8) & 0xFF,
    ])


def crc_ok(frame: bytes) -> bool:
    if len(frame) < 5:
        return False

    received = (
        frame[-2]
        | (frame[-1] << 8)
    )

    return (
        received
        == crc16_modbus(frame[:-2])
    )


# ============================================================
# 同时输出到终端和 terminal.log
# ============================================================

class FullTee:

    """
    完整输出：
    同时写终端和日志文件。
    """

    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for stream in self.streams:
            stream.write(data)
            stream.flush()

        return len(data)

    def flush(self):
        for stream in self.streams:
            stream.flush()

    def isatty(self):
        return False


class ConciseTee:

    """
    默认终端简洁显示。

    所有原始print内容仍完整写入terminal.log，
    这里只过滤屏幕显示，不影响任何采集结果。
    """

    CHANNELS = (
        "开放式超声波",
        "地电波 TEV",
        "接触式超声波",
        "特高频 UHF",
    )

    FILE_PREFIXES = (
        "完整JSON",
        "逐次采样CSV",
        "360点图谱CSV",
        "通道汇总CSV",
        "Modbus事务CSV",
        "Modbus原始日志",
        "完整终端日志",
    )

    def __init__(
        self,
        console,
        full_log,
    ):
        self.console = console
        self.full_log = full_log

        self.buffer = ""

        self.in_channel_result = False
        self.in_summary = False
        self.in_files = False

    def write(self, data):

        # ----------------------------------------------------
        # 无条件完整保存到terminal.log
        # ----------------------------------------------------

        self.full_log.write(data)
        self.full_log.flush()

        # ----------------------------------------------------
        # 屏幕按行过滤
        # ----------------------------------------------------

        self.buffer += data

        while "\n" in self.buffer:

            line, self.buffer = (
                self.buffer.split(
                    "\n",
                    1,
                )
            )

            rendered = self.render_line(
                line.rstrip("\r")
            )

            if rendered is not None:

                self.console.write(
                    rendered + "\n"
                )

                self.console.flush()

        return len(data)

    def flush(self):

        self.full_log.flush()
        self.console.flush()

    def isatty(self):
        return False

    def render_line(self, line):

        s = line.strip()

        if not s:
            return None

        # ----------------------------------------------------
        # 不显示大块分隔线
        # ----------------------------------------------------

        if (
            s.startswith("=")
            or s.startswith("#")
            or s.startswith("-")
        ):
            return None

        # ----------------------------------------------------
        # 严重错误和设备警告始终显示
        # ----------------------------------------------------

        if s.startswith(
            (
                "[FATAL ERROR]",
                "[CHANNEL ERROR]",
                "[DEVICE INFO WARN]",
                "[DEVICE INFO RAW]",
            )
        ):
            return s

        # ----------------------------------------------------
        # 总体启动信息
        # ----------------------------------------------------

        if s == (
            "G300 四通道完整局放采集"
        ):
            return s

        if s.startswith(
            (
                "结果目录：",
                "串口：",
                "波特率：",
                "设备ID：",
                "采集周期：",
                "采集顺序：",
                "[COMM]",
            )
        ):
            return s

        # ----------------------------------------------------
        # 唤醒
        # ----------------------------------------------------

        if s == "G300 唤醒":
            return "[1/6] G300 通信与唤醒"

        if s.startswith(
            "第一次唤醒应答："
        ):
            return (
                "    "
                + s
            )

        if s.startswith(
            "第二次唤醒应答："
        ):
            return (
                "    "
                + s
            )

        # ----------------------------------------------------
        # 设备信息
        # ----------------------------------------------------

        if s == "设备信息":
            return "[2/6] 设备信息"

        if s.startswith(
            (
                "硬件版本：",
                "软件版本：",
                "当前采集周期：",
                "自动休眠：",
                "灵敏度：",
                "放电最小次数阈值：",
            )
        ):
            return (
                "    "
                + s
            )

        # ----------------------------------------------------
        # 通道开始
        # ----------------------------------------------------

        if s.startswith(
            "开始采集："
        ):
            self.in_channel_result = False
            self.in_summary = False
            self.in_files = False

            channel_index = None
            channel_name = None

            for i, name in enumerate(
                self.CHANNELS,
                start=1,
            ):
                if name in s:
                    channel_index = i
                    channel_name = name
                    break

            if (
                channel_index is not None
                and channel_name is not None
            ):
                return (
                    f"[检测] "
                    f"{channel_name}：开始"
                )

            return s

        # ----------------------------------------------------
        # 逐次采样：
        # 只显示 1 / 5 / 10 / 15 / 20
        # ----------------------------------------------------

        if " Sample " in s:

            try:
                tail = s.rsplit(
                    "Sample ",
                    1,
                )[1]

                current_text, total_text = (
                    tail.split(
                        "/",
                        1,
                    )
                )

                current = int(
                    current_text.strip()
                )

                total = int(
                    total_text.strip()
                )

                show_points = {
                    1,
                    5,
                    10,
                    15,
                    total,
                }

                if current in show_points:
                    return (
                        f"    采集进度："
                        f"{current}/{total}"
                    )

            except Exception:
                pass

            return None

        # ----------------------------------------------------
        # 完整结果读取
        # ----------------------------------------------------

        if s.startswith(
            "[FULL READ]"
        ):
            return (
                "    正在读取完整诊断数据"
                "和360点图谱..."
            )

        if s.startswith(
            "[ANALYSIS]"
        ):
            return (
                "    正在等待G300"
                "最终分析..."
            )

        # ----------------------------------------------------
        # 单通道最终结果
        # ----------------------------------------------------

        if s.endswith(
            "完整结果"
        ):
            self.in_channel_result = True

            return (
                "    "
                + s
            )

        if s.startswith(
            "360点图谱"
        ):
            # 从这里开始后面360个点全部不显示到屏幕
            self.in_channel_result = False
            return None

        if self.in_channel_result:

            if s.startswith(
                (
                    "最终类型",
                    "放电判定",
                    "基本数据",
                    "周期内脉冲数",
                    "放电强度",
                    "检测耗时",
                )
            ):
                return (
                    "        "
                    + s
                )

            return None

        # ----------------------------------------------------
        # 四通道最终汇总
        # ----------------------------------------------------

        if s == "四通道最终汇总":

            self.in_summary = True
            self.in_files = False

            return (
                "[5/6] 四通道最终汇总"
            )

        if self.in_summary:

            if s.startswith(
                self.CHANNELS
            ):
                return (
                    "    "
                    + s
                )

            if s == "结果文件":

                self.in_summary = False
                self.in_files = True

                return (
                    "[6/6] 结果文件"
                )

            return None

        # ----------------------------------------------------
        # 结果文件
        # ----------------------------------------------------

        if s == "结果文件":

            self.in_files = True

            return (
                "[6/6] 结果文件"
            )

        if self.in_files:

            if s.startswith(
                self.FILE_PREFIXES
            ):
                return (
                    "    "
                    + s
                )

            return None

        return None


class ModbusError(RuntimeError):
    pass


# ============================================================
# Modbus RTU
# ============================================================

class ModbusRTU:

    def __init__(
        self,
        port,
        baud,
        slave_id,
        raw_log_path,
    ):
        self.port = port
        self.baud = baud
        self.slave_id = slave_id

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        self.raw_log = open(
            raw_log_path,
            "w",
            encoding="utf-8",
        )

        self.transactions = []

    def close(self):
        try:
            self.ser.close()
        finally:
            self.raw_log.close()

    def _read_exact(
        self,
        count,
        timeout,
    ):
        deadline = (
            time.monotonic()
            + timeout
        )

        data = bytearray()

        while (
            len(data) < count
            and time.monotonic() < deadline
        ):
            chunk = self.ser.read(
                count - len(data)
            )

            if chunk:
                data.extend(chunk)

        return bytes(data)

    def _record(
        self,
        label,
        tx,
        rx,
        valid_crc,
    ):
        item = {
            "timestamp": now_iso(),
            "label": label,
            "tx_hex": tx.hex(" "),
            "rx_hex": (
                rx.hex(" ")
                if rx
                else ""
            ),
            "rx_len": len(rx),
            "crc_ok": (
                bool(valid_crc)
                if rx
                else False
            ),
        }

        self.transactions.append(item)

        self.raw_log.write(
            f'{item["timestamp"]}'
            f' | {label}'
            f' | TX {item["tx_hex"]}'
            f' | RX '
            f'{item["rx_hex"] if item["rx_hex"] else "<EMPTY>"}'
            f' | CRC '
            f'{"OK" if item["crc_ok"] else ("INVALID" if rx else "N/A")}'
            f'\n'
        )

        self.raw_log.flush()

    def exchange(
        self,
        payload,
        expected_func,
        label,
        timeout=0.5,
        allow_no_response=False,
    ):
        tx = add_crc(payload)

        self.ser.reset_input_buffer()

        self.ser.write(tx)
        self.ser.flush()

        head = self._read_exact(
            3,
            timeout,
        )

        if len(head) < 3:
            self._record(
                label,
                tx,
                head,
                False,
            )

            if allow_no_response:
                return b""

            raise TimeoutError(
                f"{label}: 无应答"
            )

        func = head[1]

        if func & 0x80:
            tail = self._read_exact(
                2,
                timeout,
            )

            rx = head + tail

        elif func == 0x03:
            byte_count = head[2]

            tail = self._read_exact(
                byte_count + 2,
                timeout,
            )

            rx = head + tail

        elif func == 0x06:
            tail = self._read_exact(
                5,
                timeout,
            )

            rx = head + tail

        else:
            time.sleep(0.02)

            rx = (
                head
                + self.ser.read(
                    self.ser.in_waiting
                    or 0
                )
            )

        valid = crc_ok(rx)

        self._record(
            label,
            tx,
            rx,
            valid,
        )

        if not valid:
            if allow_no_response:
                return b""

            raise ModbusError(
                f"{label}: "
                f"CRC错误或响应不完整："
                f"{rx.hex(' ')}"
            )

        if (
            rx[0] != self.slave_id
            and rx[0] != 0xFF
        ):
            raise ModbusError(
                f"{label}: "
                f"设备地址异常，"
                f"期望={self.slave_id}, "
                f"收到={rx[0]}"
            )

        if rx[1] & 0x80:
            err = rx[2]

            errors = {
                0x01: "不支持的功能码",
                0x02: "寄存器地址错误",
                0x03: "数据值域错误",
                0x04: "写入失败",
                0x05: "设备休眠状态",
            }

            raise ModbusError(
                f"{label}: "
                f"Modbus异常 "
                f"0x{err:02X} "
                f"({errors.get(err, '未知异常')})"
            )

        if rx[1] != expected_func:
            raise ModbusError(
                f"{label}: "
                f"功能码异常 "
                f"0x{rx[1]:02X}"
            )

        return rx

    def read_registers(
        self,
        address,
        count,
        label,
        timeout=0.7,
        retries=2,
    ):
        last_exc = None

        for attempt in range(
            1,
            retries + 1,
        ):
            try:
                payload = bytes([
                    self.slave_id,
                    0x03,

                    (address >> 8)
                    & 0xFF,

                    address
                    & 0xFF,

                    (count >> 8)
                    & 0xFF,

                    count
                    & 0xFF,
                ])

                rx = self.exchange(
                    payload,
                    0x03,
                    (
                        f"{label} "
                        f"[try "
                        f"{attempt}/"
                        f"{retries}]"
                    ),
                    timeout=timeout,
                )

                byte_count = rx[2]

                expected_bytes = (
                    count * 2
                )

                if (
                    byte_count
                    != expected_bytes
                ):
                    raise ModbusError(
                        f"{label}: "
                        f"期望"
                        f"{expected_bytes}字节，"
                        f"实际"
                        f"{byte_count}字节"
                    )

                data = rx[
                    3:
                    3 + byte_count
                ]

                regs = []

                for i in range(
                    0,
                    len(data),
                    2,
                ):
                    regs.append(
                        (data[i] << 8)
                        | data[i + 1]
                    )

                return regs

            except Exception as exc:
                last_exc = exc

                if attempt < retries:
                    time.sleep(0.15)

        raise last_exc

    def write_register(
        self,
        address,
        value,
        label,
        timeout=0.7,
        allow_no_response=False,
    ):
        payload = bytes([
            self.slave_id,
            0x06,

            (address >> 8)
            & 0xFF,

            address
            & 0xFF,

            (value >> 8)
            & 0xFF,

            value
            & 0xFF,
        ])

        rx = self.exchange(
            payload,
            0x06,
            label,
            timeout=timeout,
            allow_no_response=(
                allow_no_response
            ),
        )

        if not rx:
            return False

        if len(rx) != 8:
            if allow_no_response:
                return False

            raise ModbusError(
                f"{label}: "
                f"写寄存器应答长度异常"
            )

        return True


# ============================================================
# 数据解析
# ============================================================

def parse_basic(regs):

    if len(regs) < 5:
        raise ValueError(
            "0x0509返回不足5个寄存器"
        )

    discharge_type = (
        regs[2] & 0xFF
    )

    sync = (
        regs[4] & 0xFF
    )

    return {
        "basic_raw":
            regs[0],

        "basic_value":
            regs[0] / 100.0,

        "pulse_count":
            regs[1],

        "discharge_type":
            discharge_type,

        "discharge_type_name":
            DISCHARGE_TYPE.get(
                discharge_type,
                "未知",
            ),

        "collection_count":
            regs[3],

        "sync":
            sync,

        "sync_name":
            SYNC_NAME.get(
                sync,
                "未知",
            ),
    }


def signed_low_byte(reg):

    value = (
        reg & 0xFF
    )

    if value >= 128:
        value -= 256

    return value


def print_spectrum(values):

    for start in range(
        0,
        360,
        30,
    ):
        chunk = values[
            start:
            start + 30
        ]

        print(
            f"相位 "
            f"{start + 1:03d}-"
            f"{start + len(chunk):03d}° : "
            +
            " ".join(
                f"{v:4d}"
                for v in chunk
            )
        )


# ============================================================
# 唤醒
# ============================================================

def wake_device(bus):

    print()
    print("=" * 80)
    print("G300 唤醒")
    print("=" * 80)

    first = bus.write_register(
        0x02F0,
        0x0002,
        "WAKE-1",
        timeout=0.15,
        allow_no_response=True,
    )

    time.sleep(0.020)

    second = bus.write_register(
        0x02F0,
        0x0002,
        "WAKE-2",
        timeout=0.60,
        allow_no_response=True,
    )

    print(
        "第一次唤醒应答：",
        first,
    )

    print(
        "第二次唤醒应答：",
        second,
    )

    time.sleep(0.25)


# ============================================================
# 设备公共信息
# ============================================================

def read_device_info(bus):

    info = {
        "hardware_version_raw": None,
        "hardware_version": None,

        "software_version_raw": None,
        "software_version": None,

        "acquisition_cycles": None,
        "auto_sleep_minutes": None,

        # 只有CRC正确时才填这里
        "sensitivity": {
            "ultrasonic": None,
            "uhf": None,
            "tev": None,
        },

        "sensitivity_crc_ok": False,

        # 如果固件返回了可解析但CRC错误的数据，
        # 仅在这里原样保存，不作为可信配置值。
        "sensitivity_unverified": None,
        "sensitivity_raw_response_hex": None,
        "sensitivity_error": None,

        "min_discharge_count": {},

        "read_warnings": [],
    }

    # ========================================================
    # 版本号
    # ========================================================

    try:
        versions = bus.read_registers(
            0x0505,
            2,
            "READ VERSION",
        )

        info["hardware_version_raw"] = versions[0]
        info["hardware_version"] = versions[0] / 100.0

        info["software_version_raw"] = versions[1]
        info["software_version"] = versions[1] / 100.0

    except Exception as exc:
        warning = (
            "版本号读取失败: "
            f"{type(exc).__name__}: {exc}"
        )

        info["read_warnings"].append(warning)

        print(
            "[DEVICE INFO WARN] "
            + warning
        )

    # ========================================================
    # 采集周期
    # ========================================================

    try:
        info["acquisition_cycles"] = (
            bus.read_registers(
                0x0540,
                1,
                "READ CYCLES",
            )[0]
        )

    except Exception as exc:
        warning = (
            "采集周期读取失败: "
            f"{type(exc).__name__}: {exc}"
        )

        info["read_warnings"].append(warning)

        print(
            "[DEVICE INFO WARN] "
            + warning
        )

    # ========================================================
    # 自动休眠
    # ========================================================

    try:
        info["auto_sleep_minutes"] = (
            bus.read_registers(
                0x0700,
                1,
                "READ AUTO SLEEP",
            )[0]
        )

    except Exception as exc:
        warning = (
            "自动休眠读取失败: "
            f"{type(exc).__name__}: {exc}"
        )

        info["read_warnings"].append(warning)

        print(
            "[DEVICE INFO WARN] "
            + warning
        )

    # ========================================================
    # 灵敏度
    #
    # 协议只定义：
    #
    #   READ 0x0600 count=3
    #
    # 返回：
    #   第1个：超声
    #   第2个：UHF
    #   第3个：TEV
    #
    # 实机 HW1.30 / FW2.92 已发现：
    #   数据部分为 6/7/7，
    #   但响应CRC错误。
    #
    # 所以：
    #   1. 仍严格验证CRC；
    #   2. CRC错误时不接受为正式值；
    #   3. 保存原始响应和可解析值供诊断；
    #   4. 不影响四通道正式采集。
    #
    # 不再尝试 READ 0x0601 / 0x0602。
    # ========================================================

    try:
        values = bus.read_registers(
            0x0600,
            3,
            "READ SENSITIVITY",
            retries=1,
        )

        info["sensitivity"] = {
            "ultrasonic": values[0],
            "uhf": values[1],
            "tev": values[2],
        }

        info["sensitivity_crc_ok"] = True

    except Exception as exc:

        info["sensitivity_error"] = (
            f"{type(exc).__name__}: {exc}"
        )

        warning = (
            "灵敏度读取返回CRC异常；"
            "该项只记录原始响应，"
            "不作为可信配置值，继续采集"
        )

        info["read_warnings"].append(warning)

        print()
        print(
            "[DEVICE INFO WARN] "
            + warning
        )

        # 尝试从刚才记录的Modbus事务中保存原始响应。
        try:
            tx_item = bus.transactions[-1]

            raw_hex = tx_item.get(
                "rx_hex",
                "",
            )

            info[
                "sensitivity_raw_response_hex"
            ] = raw_hex

            raw = bytes.fromhex(
                raw_hex
            )

            # 标准响应：
            # ID 03 06
            # data0_hi data0_lo
            # data1_hi data1_lo
            # data2_hi data2_lo
            # CRC_L CRC_H
            if (
                len(raw) == 11
                and raw[0] == bus.slave_id
                and raw[1] == 0x03
                and raw[2] == 0x06
            ):
                values = [
                    (raw[3] << 8)
                    | raw[4],

                    (raw[5] << 8)
                    | raw[6],

                    (raw[7] << 8)
                    | raw[8],
                ]

                info[
                    "sensitivity_unverified"
                ] = {
                    "ultrasonic":
                        values[0],

                    "uhf":
                        values[1],

                    "tev":
                        values[2],

                    "crc_ok":
                        False,

                    "warning":
                        (
                            "仅由CRC错误的"
                            "设备响应解析，"
                            "不能作为可信配置值"
                        ),
                }

                print(
                    "[DEVICE INFO RAW] "
                    "灵敏度原始解析："
                    f"超声={values[0]}, "
                    f"UHF={values[1]}, "
                    f"TEV={values[2]} "
                    "(CRC INVALID)"
                )

        except Exception as parse_exc:
            print(
                "[DEVICE INFO WARN] "
                "灵敏度错误响应解析失败："
                f"{parse_exc}"
            )

    # ========================================================
    # 判断放电最小次数
    #
    # 协议读取形式：
    #
    # 0x0801 = 超声
    # 0x0811 = TEV
    # 0x0831 = UHF
    #
    # 接触式超声没有单独定义该参数，
    # 与超声类型分开记录，不自行推断。
    # ========================================================

    for name, address in (
        ("ultrasonic", 0x0801),
        ("tev",        0x0811),
        ("uhf",        0x0831),
    ):

        try:
            value = bus.read_registers(
                address,
                1,
                (
                    "READ MIN DISCHARGE "
                    f"{name}"
                ),
                retries=2,
            )[0]

            info[
                "min_discharge_count"
            ][name] = value

        except Exception as exc:

            info[
                "min_discharge_count"
            ][name] = None

            warning = (
                f"{name}判断放电最小次数"
                "读取失败: "
                f"{type(exc).__name__}: "
                f"{exc}"
            )

            info[
                "read_warnings"
            ].append(warning)

            print(
                "[DEVICE INFO WARN] "
                + warning
            )

    return info


# ============================================================
# 统一采集周期
# ============================================================

def ensure_cycles(
    bus,
    desired_cycles,
):

    current = (
        bus.read_registers(
            0x0540,
            1,
            "READ CYCLES BEFORE SET",
        )[0]
    )

    if current == desired_cycles:
        return current

    print(
        f"[CONFIG] "
        f"采集周期 "
        f"{current} -> "
        f"{desired_cycles}"
    )

    bus.write_register(
        0x0540,
        desired_cycles,
        "SET CYCLES",
        timeout=2.2,
    )

    time.sleep(1.6)

    verify = (
        bus.read_registers(
            0x0540,
            1,
            "VERIFY CYCLES",
        )[0]
    )

    if verify != desired_cycles:
        raise RuntimeError(
            "采集周期设置失败："
            f"期望={desired_cycles}，"
            f"实际={verify}"
        )

    return verify


# ============================================================
# 读取完整诊断结果
# ============================================================

def read_full_result(
    bus,
    channel,
):

    # --------------------------------------------------------
    # 360点图谱
    # --------------------------------------------------------

    part1 = (
        bus.read_registers(
            0x030A,
            120,
            "READ SPECTRUM 001-120",
            timeout=1.5,
        )
    )

    part2 = (
        bus.read_registers(
            0x0382,
            120,
            "READ SPECTRUM 121-240",
            timeout=1.5,
        )
    )

    part3 = (
        bus.read_registers(
            0x03FA,
            124,
            (
                "READ SPECTRUM "
                "241-360 + META"
            ),
            timeout=1.5,
        )
    )

    spectrum = [
        signed_low_byte(v)
        for v in (
            part1
            + part2
            + part3[:120]
        )
    ]

    spectrum_type = (
        part3[123] >> 8
    ) & 0xFF

    spectrum_sync = (
        part3[123]
        & 0xFF
    )

    spectrum_meta = {
        "collection_count":
            part3[120],

        "pulse_count":
            part3[121],

        "basic_raw":
            part3[122],

        "basic_value":
            part3[122] / 100.0,

        "discharge_type":
            spectrum_type,

        "discharge_type_name":
            DISCHARGE_TYPE.get(
                spectrum_type,
                "未知",
            ),

        "sync":
            spectrum_sync,

        "sync_name":
            SYNC_NAME.get(
                spectrum_sync,
                "未知",
            ),
    }

    # --------------------------------------------------------
    # 放电概率
    # --------------------------------------------------------

    probability_regs = (
        bus.read_registers(
            0x0709,
            6,
            "READ DISCHARGE PROBABILITY",
        )
    )

    probability = {
        "discharge_type":
            probability_regs[0],

        "discharge_type_name":
            DISCHARGE_TYPE.get(
                probability_regs[0],
                "未知",
            ),

        "internal_percent":
            probability_regs[1],

        "surface_percent":
            probability_regs[2],

        "floating_percent":
            probability_regs[3],

        "corona_percent":
            probability_regs[4],

        "noise_percent":
            probability_regs[5],
    }

    # --------------------------------------------------------
    # 放电强度
    # --------------------------------------------------------

    intensity_raw = (
        bus.read_registers(
            0x0909,
            1,
            "READ DISCHARGE INTENSITY",
        )[0]
    )

    intensity = {
        "code":
            intensity_raw,

        "name":
            INTENSITY.get(
                intensity_raw,
                "未知",
            ),
    }

    # --------------------------------------------------------
    # 50 / 100 Hz相关性
    # --------------------------------------------------------

    correlation_regs = (
        bus.read_registers(
            0x0920,
            2,
            "READ 50/100HZ CORRELATION",
        )
    )

    correlation = {
        "50hz_percent":
            correlation_regs[0],

        "100hz_percent":
            correlation_regs[1],
    }

    # --------------------------------------------------------
    # 放电统计
    # --------------------------------------------------------

    stats = (
        bus.read_registers(
            0x0930,
            6,
            "READ DISCHARGE STATISTICS",
        )
    )

    statistics = {
        "discharge_count":
            stats[0],

        "background_raw":
            stats[1],

        "background_voltage_mv":
            stats[2] / 10.0,

        "max_raw":
            stats[3],

        "max_voltage_mv":
            stats[4] / 10.0,

        "multi_cycle_pulse_count":
            stats[5] / 100.0,
    }

    # --------------------------------------------------------
    # 对应通道脉冲数
    # 0x1001 / 0x1011 / 0x1021 / 0x1031
    # --------------------------------------------------------

    channel_pulse_count = (
        bus.read_registers(
            0x1000
            | channel["code"],
            1,
            (
                "READ CHANNEL PULSES "
                f'0x{channel["code"]:02X}'
            ),
        )[0]
    )

    return {
        "spectrum_360":
            spectrum,

        "spectrum_meta":
            spectrum_meta,

        "probability":
            probability,

        "intensity":
            intensity,

        "correlation":
            correlation,

        "statistics":
            statistics,

        "channel_pulse_count":
            channel_pulse_count,
    }


# ============================================================
# 按协议等待
# ============================================================

def sample_wait_seconds(
    channel,
    index,
    cycles,
    args,
):

    if (
        channel["group"]
        == "ultrasonic"
    ):
        # 协议明确指出超声最后一次采样耗时明显更长
        if index >= cycles:
            return (
                args.ultrasonic_final_wait
            )

        return (
            args.ultrasonic_wait
        )

    return (
        args.high_frequency_wait
    )


# ============================================================
# 执行一个完整通道
# ============================================================

def acquire_channel(
    bus,
    channel,
    cycles,
    args,
):

    started = time.monotonic()

    result = {
        "name":
            channel["name"],

        "name_cn":
            channel["name_cn"],

        "code":
            channel["code"],

        "code_hex":
            f'0x{channel["code"]:02X}',

        "success":
            False,

        "samples":
            [],
    }

    print()
    print("#" * 80)
    print(
        f'开始采集：'
        f'{channel["name_cn"]} '
        f'(0x{channel["code"]:02X})'
    )
    print(
        f"目标完整周期："
        f"{cycles} 次"
    )
    print("#" * 80)

    # --------------------------------------------------------
    # 每个通道必须独立清空
    # --------------------------------------------------------

    bus.write_register(
        0x0701,
        0x0001,
        (
            "CLEAR COUNT "
            f'{channel["name"]}'
        ),
        timeout=0.8,
    )

    time.sleep(0.12)

    try:
        cleared = parse_basic(
            bus.read_registers(
                0x0509,
                5,
                (
                    "READ STATUS AFTER CLEAR "
                    f'{channel["name"]}'
                ),
            )
        )

        print(
            "[CLEAR] "
            f'采集次数='
            f'{cleared["collection_count"]}, '
            f'类型='
            f'{cleared["discharge_type"]}'
            f'('
            f'{cleared["discharge_type_name"]}'
            f')'
        )

    except Exception as exc:
        print(
            "[CLEAR WARN] "
            f"{exc}"
        )

    final_basic = None

    # --------------------------------------------------------
    # 完整一个周期
    # --------------------------------------------------------

    for index in range(
        1,
        cycles + 1,
    ):
        print()
        print("-" * 80)

        print(
            f'{channel["name_cn"]} '
            f'Sample '
            f'{index}/{cycles}'
        )

        # 0x0402 = 通道
        ack = bus.write_register(
            0x0402,
            channel["code"],
            (
                f'TRIGGER '
                f'{channel["name"]} '
                f'{index}/{cycles}'
            ),
            timeout=0.20,
            allow_no_response=True,
        )

        wait_s = sample_wait_seconds(
            channel,
            index,
            cycles,
            args,
        )

        if not ack:
            print(
                "[WARN] "
                "采集命令没有收到回显；"
                "继续读取采集次数判断"
                "设备是否实际执行"
            )

        print(
            f"[WAIT] "
            f"{wait_s:.2f}s"
        )

        time.sleep(wait_s)

        # ----------------------------------------------------
        # 读取0x0509
        # ----------------------------------------------------

        status = None

        if index >= cycles:
            extra_timeout = (
                args.final_extra_timeout
            )
        else:
            extra_timeout = (
                args.sample_extra_timeout
            )

        deadline = (
            time.monotonic()
            + extra_timeout
        )

        while (
            time.monotonic()
            < deadline
        ):
            try:
                regs = (
                    bus.read_registers(
                        0x0509,
                        5,
                        (
                            "READ BASIC "
                            f'{channel["name"]} '
                            f'{index}/{cycles}'
                        ),
                        timeout=0.8,
                        retries=2,
                    )
                )

                candidate = (
                    parse_basic(regs)
                )

                status = candidate

                if (
                    candidate[
                        "collection_count"
                    ]
                    >= index
                ):
                    break

            except Exception as exc:
                print(
                    "[READ RETRY] "
                    f"{exc}"
                )

            time.sleep(0.25)

        if status is None:
            raise TimeoutError(
                f'{channel["name_cn"]} '
                f'第{index}次采集后'
                f'无法读取0x0509'
            )

        row = {
            "timestamp":
                now_iso(),

            "requested_index":
                index,

            "trigger_ack":
                bool(ack),

            **status,
        }

        result[
            "samples"
        ].append(row)

        final_basic = status

        print(
            f'基本数据       : '
            f'{status["basic_value"]:.2f} '
            f'(raw='
            f'{status["basic_raw"]})'
        )

        print(
            f'脉冲数         : '
            f'{status["pulse_count"]}'
        )

        print(
            f'放电类型       : '
            f'{status["discharge_type"]} '
            f'('
            f'{status["discharge_type_name"]}'
            f')'
        )

        print(
            f'当前采集次数   : '
            f'{status["collection_count"]}'
            f'/{cycles}'
        )

        print(
            f'同步方式       : '
            f'{status["sync"]} '
            f'('
            f'{status["sync_name"]}'
            f')'
        )

        print(
            f'触发命令回显   : '
            f'{ack}'
        )

    # --------------------------------------------------------
    # 采满但类型仍为9时继续等最终分析
    # --------------------------------------------------------

    if (
        final_basic
        and final_basic[
            "discharge_type"
        ] == 9
    ):
        print()
        print(
            "[ANALYSIS] "
            "采样已达到完整周期，"
            "等待最终分析..."
        )

        deadline = (
            time.monotonic()
            + args.final_analysis_timeout
        )

        while (
            time.monotonic()
            < deadline
        ):
            time.sleep(0.5)

            try:
                candidate = (
                    parse_basic(
                        bus.read_registers(
                            0x0509,
                            5,
                            (
                                "WAIT FINAL ANALYSIS "
                                f'{channel["name"]}'
                            ),
                            timeout=0.8,
                            retries=2,
                        )
                    )
                )

                final_basic = candidate

                if (
                    candidate[
                        "discharge_type"
                    ]
                    != 9
                ):
                    break

            except Exception as exc:
                print(
                    "[ANALYSIS RETRY] "
                    f"{exc}"
                )

    if (
        not final_basic
        or final_basic[
            "discharge_type"
        ] == 9
    ):
        raise TimeoutError(
            f'{channel["name_cn"]} '
            "未取得最终放电分析类型"
        )

    result[
        "final_basic"
    ] = final_basic

    # --------------------------------------------------------
    # 读取全部结果
    # --------------------------------------------------------

    print()
    print(
        "[FULL READ] "
        "读取360点图谱及完整诊断寄存器..."
    )

    full = read_full_result(
        bus,
        channel,
    )

    result.update(full)

    final_type = (
        final_basic[
            "discharge_type"
        ]
    )

    result[
        "final_discharge_type"
    ] = final_type

    result[
        "final_discharge_type_name"
    ] = DISCHARGE_TYPE.get(
        final_type,
        "未知",
    )

    result[
        "detected"
    ] = (
        final_type
        in (
            1,
            2,
            3,
            4,
        )
    )

    result[
        "duration_s"
    ] = round(
        time.monotonic()
        - started,
        3,
    )

    result["success"] = True

    # --------------------------------------------------------
    # 终端打印最终结果
    # --------------------------------------------------------

    print()
    print("=" * 80)
    print(
        f'{channel["name_cn"]} '
        "完整结果"
    )
    print("=" * 80)

    print(
        f'最终类型       : '
        f'{result["final_discharge_type"]} '
        f'('
        f'{result["final_discharge_type_name"]}'
        f')'
    )

    print(
        f'放电判定       : '
        f'{"检测到放电" if result["detected"] else "未检测到放电"}'
    )

    print(
        f'基本数据       : '
        f'{final_basic["basic_value"]:.2f}'
    )

    print(
        f'周期内脉冲数   : '
        f'{final_basic["pulse_count"]}'
    )

    print(
        f'通道脉冲数     : '
        f'{full["channel_pulse_count"]}'
    )

    print(
        f'放电强度       : '
        f'{full["intensity"]["code"]} '
        f'('
        f'{full["intensity"]["name"]}'
        f')'
    )

    probability = (
        full["probability"]
    )

    print(
        "放电概率       : "
        f'内部='
        f'{probability["internal_percent"]}%  '
        f'表面='
        f'{probability["surface_percent"]}%  '
        f'悬浮='
        f'{probability["floating_percent"]}%  '
        f'电晕='
        f'{probability["corona_percent"]}%  '
        f'噪声='
        f'{probability["noise_percent"]}%'
    )

    correlation = (
        full["correlation"]
    )

    print(
        f'相关性         : '
        f'50Hz='
        f'{correlation["50hz_percent"]}%  '
        f'100Hz='
        f'{correlation["100hz_percent"]}%'
    )

    stats = (
        full["statistics"]
    )

    print(
        f'放电次数       : '
        f'{stats["discharge_count"]}'
    )

    print(
        f'背景值         : '
        f'{stats["background_raw"]}'
    )

    print(
        f'背景电压       : '
        f'{stats["background_voltage_mv"]:.1f} mV'
    )

    print(
        f'最大值         : '
        f'{stats["max_raw"]}'
    )

    print(
        f'最大值电压     : '
        f'{stats["max_voltage_mv"]:.1f} mV'
    )

    print(
        f'多周期脉冲数   : '
        f'{stats["multi_cycle_pulse_count"]:.2f}'
    )

    print(
        f'检测耗时       : '
        f'{result["duration_s"]:.3f}s'
    )

    if args.print_spectrum:
        print()
        print(
            "360点图谱"
            "（每点对应1°相位，"
            "使用寄存器低字节有符号值）："
        )

        print_spectrum(
            full["spectrum_360"]
        )

    return result


# ============================================================
# 结果文件
# ============================================================

def save_results(
    result_dir,
    result,
    transactions,
):

    # 让完整JSON包含全部Modbus事务
    result[
        "modbus_transactions"
    ] = transactions

    # --------------------------------------------------------
    # 1. 完整JSON
    # --------------------------------------------------------

    full_json = (
        result_dir
        / "full_result.json"
    )

    with open(
        full_json,
        "w",
        encoding="utf-8",
    ) as f:
        json.dump(
            result,
            f,
            ensure_ascii=False,
            indent=2,
        )

    # --------------------------------------------------------
    # 2. 每次采集CSV
    # --------------------------------------------------------

    samples_csv = (
        result_dir
        / "samples.csv"
    )

    sample_fields = [
        "channel_name",
        "channel_name_cn",
        "channel_code_hex",
        "timestamp",
        "requested_index",
        "trigger_ack",
        "basic_raw",
        "basic_value",
        "pulse_count",
        "discharge_type",
        "discharge_type_name",
        "collection_count",
        "sync",
        "sync_name",
    ]

    with open(
        samples_csv,
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=sample_fields,
        )

        writer.writeheader()

        for channel in result["channels"]:

            for sample in channel.get(
                "samples",
                [],
            ):
                row = {
                    "channel_name":
                        channel["name"],

                    "channel_name_cn":
                        channel["name_cn"],

                    "channel_code_hex":
                        channel["code_hex"],
                }

                for key in sample_fields:
                    if key not in row:
                        row[key] = sample.get(
                            key
                        )

                writer.writerow(row)

    # --------------------------------------------------------
    # 3. 360点图谱
    # --------------------------------------------------------

    spectrum_csv = (
        result_dir
        / "spectrum_360.csv"
    )

    with open(
        spectrum_csv,
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            "channel_name",
            "channel_name_cn",
            "channel_code_hex",
            "phase_deg",
            "value",
        ])

        for channel in result["channels"]:

            spectrum = (
                channel.get(
                    "spectrum_360",
                    [],
                )
            )

            for phase, value in enumerate(
                spectrum,
                start=1,
            ):
                writer.writerow([
                    channel["name"],
                    channel["name_cn"],
                    channel["code_hex"],
                    phase,
                    value,
                ])

    # --------------------------------------------------------
    # 4. 通道摘要
    # --------------------------------------------------------

    summary_csv = (
        result_dir
        / "summary.csv"
    )

    fields = [
        "channel_name",
        "channel_name_cn",
        "channel_code_hex",
        "success",
        "detected",
        "final_discharge_type",
        "final_discharge_type_name",
        "basic_value",
        "pulse_count",
        "channel_pulse_count",
        "intensity_code",
        "intensity_name",
        "internal_percent",
        "surface_percent",
        "floating_percent",
        "corona_percent",
        "noise_percent",
        "corr_50hz_percent",
        "corr_100hz_percent",
        "discharge_count",
        "background_raw",
        "background_voltage_mv",
        "max_raw",
        "max_voltage_mv",
        "multi_cycle_pulse_count",
        "duration_s",
        "error",
    ]

    with open(
        summary_csv,
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fields,
        )

        writer.writeheader()

        for channel in result["channels"]:

            final_basic = (
                channel.get(
                    "final_basic"
                )
                or {}
            )

            probability = (
                channel.get(
                    "probability"
                )
                or {}
            )

            intensity = (
                channel.get(
                    "intensity"
                )
                or {}
            )

            correlation = (
                channel.get(
                    "correlation"
                )
                or {}
            )

            stats = (
                channel.get(
                    "statistics"
                )
                or {}
            )

            writer.writerow({
                "channel_name":
                    channel.get("name"),

                "channel_name_cn":
                    channel.get("name_cn"),

                "channel_code_hex":
                    channel.get("code_hex"),

                "success":
                    channel.get("success"),

                "detected":
                    channel.get("detected"),

                "final_discharge_type":
                    channel.get(
                        "final_discharge_type"
                    ),

                "final_discharge_type_name":
                    channel.get(
                        "final_discharge_type_name"
                    ),

                "basic_value":
                    final_basic.get(
                        "basic_value"
                    ),

                "pulse_count":
                    final_basic.get(
                        "pulse_count"
                    ),

                "channel_pulse_count":
                    channel.get(
                        "channel_pulse_count"
                    ),

                "intensity_code":
                    intensity.get("code"),

                "intensity_name":
                    intensity.get("name"),

                "internal_percent":
                    probability.get(
                        "internal_percent"
                    ),

                "surface_percent":
                    probability.get(
                        "surface_percent"
                    ),

                "floating_percent":
                    probability.get(
                        "floating_percent"
                    ),

                "corona_percent":
                    probability.get(
                        "corona_percent"
                    ),

                "noise_percent":
                    probability.get(
                        "noise_percent"
                    ),

                "corr_50hz_percent":
                    correlation.get(
                        "50hz_percent"
                    ),

                "corr_100hz_percent":
                    correlation.get(
                        "100hz_percent"
                    ),

                "discharge_count":
                    stats.get(
                        "discharge_count"
                    ),

                "background_raw":
                    stats.get(
                        "background_raw"
                    ),

                "background_voltage_mv":
                    stats.get(
                        "background_voltage_mv"
                    ),

                "max_raw":
                    stats.get(
                        "max_raw"
                    ),

                "max_voltage_mv":
                    stats.get(
                        "max_voltage_mv"
                    ),

                "multi_cycle_pulse_count":
                    stats.get(
                        "multi_cycle_pulse_count"
                    ),

                "duration_s":
                    channel.get(
                        "duration_s"
                    ),

                "error":
                    channel.get(
                        "error"
                    ),
            })

    # --------------------------------------------------------
    # 5. 所有Modbus事务
    # --------------------------------------------------------

    transactions_csv = (
        result_dir
        / "modbus_transactions.csv"
    )

    transaction_fields = [
        "timestamp",
        "label",
        "tx_hex",
        "rx_hex",
        "rx_len",
        "crc_ok",
    ]

    with open(
        transactions_csv,
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=transaction_fields,
        )

        writer.writeheader()

        writer.writerows(
            transactions
        )

    return {
        "full_result_json":
            str(full_json),

        "samples_csv":
            str(samples_csv),

        "spectrum_csv":
            str(spectrum_csv),

        "summary_csv":
            str(summary_csv),

        "transactions_csv":
            str(transactions_csv),
    }


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "G300四通道完整局放采集："
            "开放式超声、TEV、"
            "接触式超声、UHF"
        )
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
    )

    parser.add_argument(
        "--id",
        type=int,
        default=2,
    )

    parser.add_argument(
        "--cycles",
        type=int,
        default=20,
    )

    parser.add_argument(
        "--channels",
        default="all",
        help=(
            "检测类型："
            "open_ultrasonic, tev, "
            "contact_ultrasonic, uhf, all；"
            "也支持逗号组合"
        ),
    )

    parser.add_argument(
        "--output-root",
        default=str(
            Path.home()
            / "ros2_ws"
            / "g300_results"
        ),
    )

    parser.add_argument(
        "--ultrasonic-wait",
        type=float,
        default=0.20,
    )

    parser.add_argument(
        "--ultrasonic-final-wait",
        type=float,
        default=8.5,
    )

    parser.add_argument(
        "--high-frequency-wait",
        type=float,
        default=2.2,
    )

    parser.add_argument(
        "--sample-extra-timeout",
        type=float,
        default=3.0,
    )

    parser.add_argument(
        "--final-extra-timeout",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--final-analysis-timeout",
        type=float,
        default=12.0,
    )

    parser.add_argument(
        "--no-print-spectrum",
        dest="print_spectrum",
        action="store_false",
        help=(
            "终端不打印360点图谱，"
            "但文件仍完整保存"
        ),
    )

    parser.set_defaults(
        print_spectrum=True
    )

    parser.add_argument(
        "--verbose-terminal",
        action="store_true",
        help=(
            "终端显示全部详细采样过程；"
            "默认使用简洁模式"
        ),
    )

    args = parser.parse_args()

    # ========================================================
    # 检测通道选择
    # ========================================================

    channel_map = {
        item["name"]: item
        for item in CHANNELS
    }

    raw_channels = str(
        args.channels
    ).strip().lower()

    if not raw_channels:
        raw_channels = "all"

    if raw_channels == "all":
        selected_channels = list(
            CHANNELS
        )
    else:
        requested = [
            item.strip()
            for item in raw_channels.split(",")
            if item.strip()
        ]

        invalid = [
            item
            for item in requested
            if item not in channel_map
        ]

        if invalid:
            raise SystemExit(
                "不支持的检测类型："
                + ", ".join(invalid)
                + "\n允许："
                "open_ultrasonic, "
                "tev, "
                "contact_ultrasonic, "
                "uhf, "
                "all"
            )

        # 去重但保持用户给定顺序
        seen = set()
        selected_channels = []

        for item in requested:
            if item in seen:
                continue

            seen.add(item)
            selected_channels.append(
                channel_map[item]
            )

    selected_channel_names = [
        item["name"]
        for item in selected_channels
    ]

    if not (
        10
        <= args.cycles
        <= 50
    ):
        raise SystemExit(
            "--cycles必须为10~50"
        )

    run_id = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    result_dir = (
        Path(
            args.output_root
        ).expanduser()
        / run_id
    )

    result_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    terminal_path = (
        result_dir
        / "terminal.log"
    )

    terminal_file = open(
        terminal_path,
        "w",
        encoding="utf-8",
    )

    original_stdout = sys.stdout
    original_stderr = sys.stderr

    if args.verbose_terminal:

        # 调试模式：
        # 屏幕和terminal.log都显示全部内容。
        sys.stdout = FullTee(
            original_stdout,
            terminal_file,
        )

    else:

        # 正常模式：
        # 屏幕只显示流程和摘要，
        # terminal.log仍保存全部详细内容。
        sys.stdout = ConciseTee(
            original_stdout,
            terminal_file,
        )

    # stderr始终完整显示并保存
    sys.stderr = FullTee(
        original_stderr,
        terminal_file,
    )

    raw_log_path = (
        result_dir
        / "modbus_raw.log"
    )

    bus = None

    result = {
        "run_id":
            run_id,

        "started_at":
            now_iso(),

        "finished_at":
            None,

        "success":
            False,

        "device": {
            "port":
                args.port,

            "baud":
                args.baud,

            "format":
                "8N1",

            "slave_id":
                args.id,
        },

        "requested_cycles":
            args.cycles,

        "channels":
            [],

        "output_dir":
            str(result_dir),
    }

    try:

        print("=" * 80)
        print(
            "G300 四通道完整局放采集"
        )
        print("=" * 80)

        print(
            f"结果目录："
            f"{result_dir}"
        )

        print(
            f"串口："
            f"{args.port}"
        )

        print(
            f"波特率："
            f"{args.baud}"
        )

        print(
            f"设备ID："
            f"{args.id}"
        )

        print(
            f"采集周期："
            f"{args.cycles}"
        )

        print(
            "采集顺序："
            "开放式超声0x01"
            " -> TEV0x11"
            " -> 接触式超声0x21"
            " -> UHF0x31"
        )

        bus = ModbusRTU(
            args.port,
            args.baud,
            args.id,
            raw_log_path,
        )

        # ----------------------------------------------------
        # 通信确认
        # ----------------------------------------------------

        device_id = (
            bus.read_registers(
                0x0003,
                1,
                "READ DEVICE ID",
            )[0]
            & 0xFF
        )

        print()
        print(
            "[COMM] "
            f"G300设备ID="
            f"{device_id}"
        )

        wake_device(bus)

        # ----------------------------------------------------
        # 设备信息
        # ----------------------------------------------------

        device_info = (
            read_device_info(bus)
        )

        result[
            "device"
        ].update(
            device_info
        )

        print()
        print("=" * 80)
        print("设备信息")
        print("=" * 80)

        hw = device_info.get(
            "hardware_version"
        )

        sw = device_info.get(
            "software_version"
        )

        print(
            "硬件版本："
            + (
                f"{hw:.2f}"
                if hw is not None
                else "读取失败"
            )
        )

        print(
            "软件版本："
            + (
                f"{sw:.2f}"
                if sw is not None
                else "读取失败"
            )
        )

        print(
            f'当前采集周期：'
            f'{device_info["acquisition_cycles"]}'
        )

        auto_sleep = device_info.get(
            "auto_sleep_minutes"
        )

        print(
            "自动休眠："
            + (
                f"{auto_sleep} min"
                if auto_sleep is not None
                else "读取失败"
            )
        )

        sensitivity = device_info[
            "sensitivity"
        ]

        if device_info.get(
            "sensitivity_crc_ok"
        ):
            print(
                "灵敏度："
                f'超声={sensitivity["ultrasonic"]}, '
                f'UHF={sensitivity["uhf"]}, '
                f'TEV={sensitivity["tev"]} '
                "(CRC OK)"
            )

        elif device_info.get(
            "sensitivity_unverified"
        ):
            raw_s = device_info[
                "sensitivity_unverified"
            ]

            print(
                "灵敏度："
                f'超声={raw_s["ultrasonic"]}, '
                f'UHF={raw_s["uhf"]}, '
                f'TEV={raw_s["tev"]} '
                "(CRC INVALID，仅记录原始值)"
            )

        else:
            print(
                "灵敏度：读取失败"
            )

        print(
            "放电最小次数阈值："
            f'{device_info["min_discharge_count"]}'
        )

        actual_cycles = (
            ensure_cycles(
                bus,
                args.cycles,
            )
        )

        result[
            "actual_cycles"
        ] = actual_cycles

        # ----------------------------------------------------
        # 四通道
        # ----------------------------------------------------

        for channel in selected_channels:

            try:
                channel_result = (
                    acquire_channel(
                        bus,
                        channel,
                        actual_cycles,
                        args,
                    )
                )

            except Exception as exc:

                channel_result = {
                    "name":
                        channel["name"],

                    "name_cn":
                        channel["name_cn"],

                    "code":
                        channel["code"],

                    "code_hex":
                        (
                            f'0x'
                            f'{channel["code"]:02X}'
                        ),

                    "success":
                        False,

                    "error":
                        (
                            f"{type(exc).__name__}: "
                            f"{exc}"
                        ),
                }

                print()
                print("!" * 80)

                print(
                    "[CHANNEL ERROR] "
                    f'{channel["name_cn"]}: '
                    f'{channel_result["error"]}'
                )

                print(
                    "继续采集下一通道。"
                )

                print("!" * 80)

            result[
                "channels"
            ].append(
                channel_result
            )

        result["success"] = all(
            channel.get("success")
            for channel
            in result["channels"]
        )

        result[
            "finished_at"
        ] = now_iso()

        # ----------------------------------------------------
        # 汇总
        # ----------------------------------------------------

        print()
        print("=" * 80)
        print("四通道最终汇总")
        print("=" * 80)

        for channel in result[
            "channels"
        ]:

            if channel.get(
                "success"
            ):
                print(
                    f'{channel["name_cn"]:<14} '
                    f'{channel["code_hex"]} | '
                    f'type='
                    f'{channel["final_discharge_type"]} '
                    f'('
                    f'{channel["final_discharge_type_name"]}'
                    f') | '
                    f'detected='
                    f'{channel["detected"]} | '
                    f'basic='
                    f'{channel["final_basic"]["basic_value"]:.2f} | '
                    f'pulses='
                    f'{channel["final_basic"]["pulse_count"]} | '
                    f'intensity='
                    f'{channel["intensity"]["name"]}'
                )

            else:
                print(
                    f'{channel["name_cn"]:<14} '
                    f'{channel["code_hex"]} | '
                    f'FAILED | '
                    f'{channel.get("error", "")}'
                )

    except Exception as exc:

        result[
            "fatal_error"
        ] = (
            f"{type(exc).__name__}: "
            f"{exc}"
        )

        result[
            "finished_at"
        ] = now_iso()

        print()
        print(
            "[FATAL ERROR] "
            f'{result["fatal_error"]}'
        )

    finally:

        if bus is not None:
            transactions = list(
                bus.transactions
            )

            bus.close()

        else:
            transactions = []

        paths = save_results(
            result_dir,
            result,
            transactions,
        )

        print()
        print("=" * 80)
        print("结果文件")
        print("=" * 80)

        print(
            "完整JSON         : "
            f'{paths["full_result_json"]}'
        )

        print(
            "逐次采样CSV      : "
            f'{paths["samples_csv"]}'
        )

        print(
            "360点图谱CSV     : "
            f'{paths["spectrum_csv"]}'
        )

        print(
            "通道汇总CSV      : "
            f'{paths["summary_csv"]}'
        )

        print(
            "Modbus事务CSV    : "
            f'{paths["transactions_csv"]}'
        )

        print(
            "Modbus原始日志   : "
            f'{raw_log_path}'
        )

        print(
            "完整终端日志     : "
            f'{terminal_path}'
        )

        sys.stdout = (
            original_stdout
        )

        sys.stderr = (
            original_stderr
        )

        terminal_file.close()

    if result.get("success"):
        return 0

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
