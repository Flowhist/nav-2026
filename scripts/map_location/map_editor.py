#!/usr/bin/env python3
"""Launch the Finav local map editor on the loopback interface."""

from __future__ import annotations

import argparse
import os
import threading
import urllib.parse
import webbrowser
from pathlib import Path

from editor_server import EditorApplication, create_server
from location_utils import resolve_maps_dir


def _find_planner_config() -> Path:
    script = Path(__file__).resolve()
    repo = os.environ.get("FINAV_REPO_DIR", "").strip()
    candidates = []
    if repo:
        candidates.append(Path(repo) / "config" / "path_plan.yaml")
    candidates.append(script.parents[2] / "config" / "path_plan.yaml")
    for ancestor in script.parents:
        candidates.append(ancestor / "share" / "finav" / "config" / "path_plan.yaml")
    return next((path for path in candidates if path.is_file()), candidates[0])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Finav 本地地图编辑器")
    parser.add_argument("--maps-dir", help="地图根目录（默认自动探测）")
    parser.add_argument("--map", dest="initial_map", help="启动后优先打开的地图")
    parser.add_argument("--port", type=int, default=8765, help="本地端口，默认 8765")
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    maps_dir = Path(args.maps_dir or resolve_maps_dir()).resolve()
    if not maps_dir.is_dir():
        raise SystemExit("未找到 maps 目录，请通过 --maps-dir 指定。")
    web_dir = Path(__file__).resolve().parent / "editor_web"
    if not web_dir.is_dir():
        raise SystemExit(f"未找到编辑器前端资源：{web_dir}")
    application = EditorApplication(maps_dir, web_dir, _find_planner_config())
    try:
        server = create_server("127.0.0.1", args.port, application)
    except OSError as error:
        raise SystemExit(f"无法监听本地端口 {args.port}：{error}") from error
    query = ""
    if args.initial_map:
        query = "?" + urllib.parse.urlencode({"map": args.initial_map})
    url = f"http://127.0.0.1:{server.server_port}/{query}"
    print(f"Finav 地图编辑器已启动：{url}", flush=True)
    print("仅监听本机回环地址；按 Ctrl+C 退出。", flush=True)
    if not args.no_browser:
        opener = threading.Timer(0.4, webbrowser.open, args=(url,))
        opener.daemon = True
        opener.start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
