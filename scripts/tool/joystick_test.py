#!/usr/bin/env python3
"""
摇杆测试脚本 —— 实时显示摇杆状态和对应的控制指令（前后左右 + 速度档位）。

用法：
    python3 scripts/tool/joystick_test.py
    python3 scripts/tool/joystick_test.py /dev/input/js1

按 Ctrl+C 退出。
"""

import struct
import sys
import os
import math
import time
import argparse

def _find_config(name: str) -> str:
    """查找配置文件路径，优先项目 config/ 目录，回退到脚本同目录。"""
    # 从脚本位置: scripts/tool/joystick_test.py -> ../../config/<name>
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(script_dir, "..", "..", "config", name),
        os.path.join(os.getcwd(), "config", name),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return ""


def load_config():
    """从 config/joy.yaml + config/base_control.yaml 读取摇杆参数。"""
    cfg = {
        "linear_axis": 0,
        "linear_direction": -1.0,
        "angular_axis": 1,
        "angular_direction": 1.0,
        "dead_zone": 0.05,
        "sat_zone": 1.0,
        "speed_split": 0.5,
        "vel_low": 0.2,
        "vel_high": 0.4,
        "rot_low": 10.0,
        "rot_high": 25.0,
        "dev": "/dev/input/js0",
        "enabled": True,
    }

    try:
        import yaml
    except ImportError:
        print("⚠ 未安装 PyYAML，使用脚本内置默认参数")
        return cfg

    # 读 joy.yaml
    joy_path = _find_config("joy.yaml")
    if joy_path:
        try:
            with open(joy_path, "r") as f:
                data = yaml.safe_load(f) or {}
            jc = (data.get("joy_control", {}) or {}).get("ros__parameters", {}) or {}
            jn = (data.get("joy_node", {}) or {}).get("ros__parameters", {}) or {}
            if isinstance(jc, dict):
                for key in ("linear_axis", "linear_direction", "angular_axis",
                            "angular_direction", "dead_zone", "sat_zone", "speed_split"):
                    if key in jc:
                        cfg[key] = float(jc[key]) if "direction" in key or "zone" in key or "split" in key else int(jc[key])
            if isinstance(jn, dict) and "dev" in jn:
                cfg["dev"] = str(jn["dev"])
            if isinstance(jc, dict) and "enabled" in jc:
                cfg["enabled"] = bool(jc["enabled"])
        except Exception as e:
            print(f"⚠ 读取 {joy_path} 失败: {e}")

    # 读 base_control.yaml
    base_path = _find_config("base_control.yaml")
    if base_path:
        try:
            with open(base_path, "r") as f:
                data = yaml.safe_load(f) or {}
            jc = (data.get("joy_control", {}) or {}).get("ros__parameters", {}) or {}
            if isinstance(jc, dict):
                for key, target in (("js_vel_low", "vel_low"),
                                    ("js_vel_high", "vel_high"),
                                    ("js_rot_low", "rot_low"),
                                    ("js_rot_high", "rot_high")):
                    if key in jc:
                        cfg[target] = float(jc[key])
        except Exception as e:
            print(f"⚠ 读取 {base_path} 失败: {e}")

    return cfg

# ── joystick event 结构体 (Linux js_event) ──
# struct js_event {
#     uint32_t time;   // 4 bytes
#     int16_t  value;  // 2 bytes
#     uint8_t  type;   // 1 byte
#     uint8_t  number; // 1 byte
# };  // total 8 bytes
EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

# event type
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02

# 轴数量上限
MAX_AXES = 8


def scale_axis(raw: float, direction: float, dead: float, sat: float) -> float:
    """归一化轴值：死区内为0，饱和区外为±1，中间线性映射。"""
    signed = raw * direction
    mag = abs(signed)
    if mag <= dead:
        return 0.0
    if mag >= sat:
        return math.copysign(1.0, signed)
    return math.copysign((mag - dead) / (sat - dead), signed)


def tiered_speed(value: float, low: float, high: float, split: float) -> float:
    """根据归一化值决定速度档位。"""
    mag = abs(value)
    if mag <= 0.0:
        return 0.0
    speed = low if mag < split else high
    return math.copysign(speed, value)


def direction_label(linear: float, angular: float) -> str:
    """将线速度和角速度映射为人类可读方向。"""
    fwd = linear > 0.001
    bwd = linear < -0.001
    left = angular > 0.001
    right = angular < -0.001

    if fwd and left:
        return "前进左转"
    if fwd and right:
        return "前进右转"
    if bwd and left:
        return "后退左转"
    if bwd and right:
        return "后退右转"
    if fwd:
        return "前进 ↑"
    if bwd:
        return "后退 ↓"
    if left:
        return "左转 ←"
    if right:
        return "右转 →"
    return "停止 ·"


def gear_label(norm: float, split: float) -> str:
    """档位标签。"""
    mag = abs(norm)
    if mag <= 0.0:
        return "---"
    return "低速" if mag < split else "高速"


def main():
    parser = argparse.ArgumentParser(description="USB HID 摇杆测试工具")
    parser.add_argument(
        "device",
        nargs="?",
        default=None,
        help="摇杆设备路径 (默认从 joy.yaml 读取)",
    )
    args = parser.parse_args()

    # ── 从 YAML 加载配置 ──
    cfg = load_config()

    dev_path = args.device or cfg["dev"]
    if not os.path.exists(dev_path):
        print(f"❌ 设备不存在: {dev_path}")
        print("   请检查连接或用命令行参数指定路径")
        sys.exit(1)

    if not cfg["enabled"]:
        print("⚠ joy.yaml 中 enabled=false，但仍继续测试...")

    # ── 配置变量（从 YAML 读取） ──
    linear_axis = cfg["linear_axis"]
    linear_dir = cfg["linear_direction"]
    angular_axis = cfg["angular_axis"]
    angular_dir = cfg["angular_direction"]
    dead_zone = cfg["dead_zone"]
    sat_zone = cfg["sat_zone"]
    speed_split = cfg["speed_split"]
    vel_low = cfg["vel_low"]
    vel_high = cfg["vel_high"]
    rot_low = cfg["rot_low"]
    rot_high = cfg["rot_high"]

    # 打开设备（二进制只读非阻塞）
    try:
        fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
    except PermissionError:
        print(f"❌ 权限不足: {dev_path}\n   试试 sudo 或把用户加入 input 组")
        sys.exit(1)

    axes = [0.0] * MAX_AXES

    print("=" * 52)
    print(f"  摇杆测试 | 设备: {dev_path} | 刷新率: ~25Hz")
    print(f"  线性轴[{linear_axis}] 方向={linear_dir:+}  旋转轴[{angular_axis}] 方向={angular_dir:+}")
    print(f"  死区={dead_zone}  饱和区={sat_zone}  档位分界={speed_split}")
    print(f"  低速: {vel_low}m/s  {rot_low}°/s    高速: {vel_high}m/s  {rot_high}°/s")
    print("=" * 52)
    print()
    print("  物理轴位(左:前后 右:旋转)  方向          线速度        角速度      原始轴值           档位")
    print("  " + "-" * 48)

    last_display = time.monotonic()
    display_interval = 0.04  # ~25Hz

    try:
        while True:
            # 非阻塞读取 joystick 事件
            try:
                data = os.read(fd, EVENT_SIZE * 8)
            except BlockingIOError:
                data = b""

            for i in range(0, len(data), EVENT_SIZE):
                chunk = data[i : i + EVENT_SIZE]
                if len(chunk) < EVENT_SIZE:
                    break
                _, value, etype, number = struct.unpack(EVENT_FORMAT, chunk)
                if etype == JS_EVENT_AXIS and number < MAX_AXES:
                    axes[number] = value / 32767.0

            # 定时刷新显示
            now = time.monotonic()
            if now - last_display < display_interval:
                time.sleep(0.001)
                continue
            last_display = now

            # 计算当前的控制输出
            raw_lin = axes[linear_axis] if linear_axis < MAX_AXES else 0.0
            raw_ang = axes[angular_axis] if angular_axis < MAX_AXES else 0.0

            lin_norm = scale_axis(raw_lin, linear_dir, dead_zone, sat_zone)
            ang_norm = scale_axis(raw_ang, angular_dir, dead_zone, sat_zone)

            lin_speed = tiered_speed(lin_norm, vel_low, vel_high, speed_split)
            ang_speed_rad = tiered_speed(
                ang_norm,
                math.radians(rot_low),
                math.radians(rot_high),
                speed_split,
            )
            ang_speed_deg = math.degrees(ang_speed_rad)

            # 绘制轴可视化条 — 使用原始物理轴值，直观反映摇杆推动方向
            def bar(value, width=10):
                pos = int(round((value + 1.0) / 2.0 * (width - 1)))
                pos = max(0, min(width - 1, pos))
                chars = ["·"] * width
                chars[width // 2] = "|"
                chars[pos] = "█"
                return "".join(chars)

            lin_bar = bar(raw_lin, 10)
            ang_bar = bar(raw_ang, 10)

            max_norm = max(abs(lin_norm), abs(ang_norm))
            gear = gear_label(max_norm, speed_split)

            status = (
                f"  {lin_bar} {ang_bar}  "
                f"{direction_label(lin_speed, ang_speed_rad):<10}  "
                f"{lin_speed:+7.3f}m/s  {ang_speed_deg:+7.2f}°/s  "
                f"原始线={raw_lin:+6.2f} 原始旋={raw_ang:+6.2f}  "
                f"{gear}"
            )
            sys.stdout.write("\r\033[K" + status)
            sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\n已退出。")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
