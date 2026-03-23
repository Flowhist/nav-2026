#!/usr/bin/env python3
"""
手动保存地图脚本

用法：
    python3 scripts/map/manual_save_map.py --name my_map

说明：
1) 保存到项目 maps/<地图名>/<地图名>.{pgm,yaml,data,posegraph}
2) 复用 Web 保存按钮同款逻辑：
   - /slam_toolbox/save_map
   - /slam_toolbox/serialize_map
3) 若 save_map 服务返回失败，自动回退 map_saver_cli 再尝试输出 pgm/yaml
"""

import argparse
import os
import re
import subprocess
import sys
from datetime import datetime


def sanitize_name(name: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_-]", "", name or "")
    return safe or f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"


def run_cmd(cmd, timeout=20):
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def save_map_success(text: str) -> bool:
    """兼容 slam_toolbox SaveMap 返回：result=0 / result: 0 / result: true"""
    t = text or ""

    # bool 风格
    if re.search(r"result\s*[:=]\s*true", t, re.IGNORECASE):
        return True
    if re.search(r"result\s*[:=]\s*false", t, re.IGNORECASE):
        return False

    # 数值风格（SaveMap: 0 表示成功）
    m = re.search(r"result\s*[:=]\s*(-?\d+)", t, re.IGNORECASE)
    if m:
        try:
            code = int(m.group(1))
            return code == 0
        except ValueError:
            return False

    return False


def serialize_success(text: str) -> bool:
    """兼容 SerializePoseGraph 返回：result=true / result: true / result=1/0"""
    t = text or ""

    if re.search(r"result\s*[:=]\s*true", t, re.IGNORECASE):
        return True
    if re.search(r"result\s*[:=]\s*false", t, re.IGNORECASE):
        return False

    m = re.search(r"result\s*[:=]\s*(-?\d+)", t, re.IGNORECASE)
    if m:
        try:
            return int(m.group(1)) == 0
        except ValueError:
            return False

    return False


def ensure_pgm_yaml_with_fallback(save_path: str) -> bool:
    cmd = [
        "ros2",
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        save_path,
        "--ros-args",
        "-r",
        "map:=/map",
        "-p",
        "map_subscribe_transient_local:=true",
        "-p",
        "save_map_timeout:=5.0",
    ]
    print("[WARN] save_map 服务未成功，回退到 map_saver_cli ...")
    rc, out, err = run_cmd(cmd, timeout=20)
    if rc != 0:
        print("[ERROR] map_saver_cli 失败")
        if err:
            print(err)
        if out:
            print(out)
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description="手动保存当前 SLAM 地图")
    parser.add_argument("-n", "--name", default="manual_map", help="地图名（默认: manual_map）")
    args = parser.parse_args()

    safe_name = sanitize_name(args.name)

    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    maps_dir = os.path.join(pkg_dir, "maps")
    map_folder = os.path.join(maps_dir, safe_name)
    save_path = os.path.join(map_folder, safe_name)
    os.makedirs(map_folder, exist_ok=True)

    print(f"[INFO] 地图将保存到: {save_path}")

    cmd_save_map = [
        "ros2",
        "service",
        "call",
        "/slam_toolbox/save_map",
        "slam_toolbox/srv/SaveMap",
        f"{{name: {{data: '{save_path}'}}}}",
    ]

    cmd_serialize = [
        "ros2",
        "service",
        "call",
        "/slam_toolbox/serialize_map",
        "slam_toolbox/srv/SerializePoseGraph",
        f"{{filename: '{save_path}'}}",
    ]

    print("[INFO] 调用 /slam_toolbox/save_map ...")
    rc1, out1, err1 = run_cmd(cmd_save_map)
    if rc1 != 0:
        print("[ERROR] save_map 调用失败（服务调用层）")
        if err1:
            print(err1)
        if out1:
            print(out1)
        sys.exit(1)

    save_ok = save_map_success(out1)
    if not save_ok:
        if out1:
            print(out1)
        if err1:
            print(err1)
        if not ensure_pgm_yaml_with_fallback(save_path):
            sys.exit(2)

    print("[INFO] 调用 /slam_toolbox/serialize_map ...")
    rc2, out2, err2 = run_cmd(cmd_serialize)
    if rc2 != 0:
        print("[ERROR] serialize_map 调用失败（服务调用层）")
        if err2:
            print(err2)
        if out2:
            print(out2)
        sys.exit(3)

    if not serialize_success(out2):
        print("[ERROR] serialize_map 服务返回失败")
        if out2:
            print(out2)
        if err2:
            print(err2)
        sys.exit(4)

    expected = [
        f"{save_path}.pgm",
        f"{save_path}.yaml",
        f"{save_path}.data",
        f"{save_path}.posegraph",
    ]
    missing = [f for f in expected if not os.path.exists(f)]

    print("[OK] 地图保存流程完成")
    print(f"[OK] 输出目录: {map_folder}")
    if missing:
        print("[WARN] 以下文件未生成:")
        for m in missing:
            print(f"  - {m}")
    else:
        print("[OK] 已生成 pgm/yaml/data/posegraph")


if __name__ == "__main__":
    main()
