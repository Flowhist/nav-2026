#!/usr/bin/env python3
"""
共享地图工具：maps 目录解析、地图发现、地点读写、运行地图检测。

被 register_location / location_visualizer / nav_to_location 共用。
"""

import os
import subprocess
from pathlib import Path
from typing import Dict, List, Optional

import yaml


def resolve_maps_dir() -> str:
    """返回 maps/ 目录绝对路径，优先级：环境变量 → repo 结构 → 安装结构。"""
    env_maps = os.environ.get("FINAV_MAPS_DIR", "").strip()
    if env_maps and os.path.isdir(env_maps):
        return env_maps
    repo = os.environ.get("FINAV_REPO_DIR", "").strip()
    if repo:
        candidate = os.path.join(repo, "maps")
        if os.path.isdir(candidate):
            return candidate
    # 从脚本路径向上探测
    exe = Path(__file__).resolve()
    for ancestor in exe.parents:
        src_maps = ancestor / "src" / "finav" / "maps"
        if src_maps.is_dir():
            return str(src_maps)
    for _ in range(6):
        candidate = exe.parent / "maps"
        if candidate.is_dir():
            return str(candidate)
        exe = exe.parent
    exe = Path(__file__).resolve()
    for ancestor in exe.parents:
        share_maps = ancestor / "share" / "finav" / "maps"
        if share_maps.is_dir():
            return str(share_maps)
    return ""


def discover_maps(maps_dir: str) -> List[str]:
    """列出 maps_dir 下所有有效地图名（含 .yaml + .pgm）。"""
    names = []
    if not maps_dir or not os.path.isdir(maps_dir):
        return names
    for name in sorted(os.listdir(maps_dir)):
        d = os.path.join(maps_dir, name)
        if not os.path.isdir(d):
            continue
        yml = os.path.join(d, f"{name}.yaml")
        pgm = os.path.join(d, f"{name}.pgm")
        if os.path.exists(yml) and os.path.exists(pgm):
            names.append(name)
    return names


def discover_maps_with_locations(maps_dir: str) -> List[str]:
    """列出 maps_dir 下包含 .locations.yaml 的地图。"""
    names = []
    if not maps_dir or not os.path.isdir(maps_dir):
        return names
    for name in sorted(os.listdir(maps_dir)):
        d = os.path.join(maps_dir, name)
        if not os.path.isdir(d):
            continue
        loc = os.path.join(d, f"{name}.locations.yaml")
        if os.path.exists(loc):
            names.append(name)
    return names


def load_locations(path: Path) -> Dict[str, Dict[str, float]]:
    """从 .locations.yaml 读取地点字典。"""
    if not path.exists():
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError:
        return {}
    entries = data.get("locations", {})
    if not isinstance(entries, dict):
        return {}
    result: Dict[str, Dict[str, float]] = {}
    for name, loc in entries.items():
        if isinstance(loc, dict) and "x" in loc and "y" in loc:
            result[str(name)] = {
                "x": float(loc["x"]),
                "y": float(loc["y"]),
                "yaw_deg": float(loc.get("yaw_deg", 0.0)),
            }
    return result


def save_locations(path: Path, locations: Dict[str, Dict[str, float]]) -> None:
    """写入 .locations.yaml。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    out = {"locations": locations}
    path.write_text(
        yaml.dump(out, allow_unicode=True, default_flow_style=False),
        encoding="utf-8",
    )


def detect_running_map_file(maps_dir: str) -> str:
    """尝试从当前运行的 ROS 节点参数中自动检测 map_file。"""
    import subprocess as _sp

    def _ros2_param_get(node_name: str, param_name: str) -> str:
        try:
            proc = _sp.run(
                ["ros2", "param", "get", node_name, param_name],
                check=False,
                stdout=_sp.PIPE,
                stderr=_sp.DEVNULL,
                text=True,
                timeout=2.0,
            )
        except (OSError, _sp.TimeoutExpired):
            return ""
        if proc.returncode != 0:
            return ""
        text = proc.stdout.strip()
        marker = "value is:"
        if marker in text:
            text = text.split(marker, 1)[1].strip()
        return text.strip().strip("'\"")

    # 优先查 nav_bridge / location_visualizer 自身的 map_file 参数
    for node_name in ("/nav_bridge", "/location_visualizer"):
        mf = _ros2_param_get(node_name, "map_file")
        if mf and _locations_file_exists(mf, maps_dir):
            return mf

    # 从 slam_toolbox 的 map_file_name 反推
    map_path = _ros2_param_get("/slam_toolbox", "map_file_name")
    if map_path:
        p = Path(map_path)
        if p.name and p.parent.name == p.name:
            mf = p.name
        elif p.suffix in (".yaml", ".posegraph"):
            mf = p.stem
        else:
            mf = p.name
        if _locations_file_exists(mf, maps_dir):
            return mf

    return ""


def _locations_file_exists(map_file: str, maps_dir: str) -> bool:
    return bool(map_file) and (
        Path(maps_dir) / map_file / f"{map_file}.locations.yaml"
    ).exists()
