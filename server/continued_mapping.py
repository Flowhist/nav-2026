"""Validation and session gates for continued mapping HTTP operations."""

import time

from web_support import RequestError


def validate_mapping_map(app, name):
    target = app._resolve_map_dir(name) if isinstance(name, str) else None
    if target is None or not name or name.startswith("-"):
        raise RequestError(400, "请选择有效的续建地图")
    for suffix in ("yaml", "pgm", "posegraph", "data"):
        path = target / f"{name}.{suffix}"
        if path.is_symlink() or not path.is_file() or path.stat().st_size == 0:
            raise RequestError(400, f"续建地图缺少有效的 .{suffix} 文件")
    return target


def require_continued_mapping(app, stages):
    runtime = app.runtime.snapshot()
    mapping = runtime.get("mapping", {})
    name = mapping.get("launch_args", {}).get("map_file", "")
    if not mapping.get("running") or mapping.get("stopping") or not name:
        raise RequestError(409, "请先加载续建地图")
    status = app.state.snapshot().get("continued_mapping", {})
    expected = str((app.maps_dir / name / name).resolve())
    received = status.get("updated_at", 0)
    if (status.get("prefix") != expected or time.time() - received > 3.0
            or received < (mapping.get("started_at") or 0)
            or status.get("stage") not in stages):
        raise RequestError(409, "续建节点尚未就绪或定位尚未完成，请等待状态更新")
    return name, runtime


def start_mapping(app, body):
    name = body.get("map_file", "")
    if not isinstance(name, str):
        raise RequestError(400, "地图名必须是字符串")
    if name:
        validate_mapping_map(app, name)
    if app.state.snapshot().get("map_save", {}).get("state") == "running":
        raise RequestError(409, "请等待地图保存完成")
    current = app.runtime.snapshot().get("mapping", {})
    if current.get("running") or current.get("stopping"):
        raise RequestError(409, "请先结束当前建图会话")
    app.state.update_status({"continued_mapping": {"stage": "loading" if name else "idle",
                            "prefix": "", "updated_at": 0}})
    args = {"map_file": name, "maps_dir": str(app.maps_dir.resolve())} if name else {}
    return app.runtime.start("mapping", launch_args=args)


def mapping_action(app, action, body):
    name, runtime = require_continued_mapping(app, {"localizing", "ready"})
    relocate = runtime.get("relocate", {})
    if relocate.get("running") or relocate.get("stopping"):
        raise RequestError(409, "请等待重定位结束")
    if action == "initialpose":
        app.state.update_status({"continued_mapping": {"stage": "localizing"}})
        app.bridge.command({"type": "set_initialpose", **{key: body[key] for key in ("x", "y", "yaw_deg")}})
    elif action == "relocate":
        return app.runtime.run_relocate(params={"map_file": name, "maps_dir": str(app.maps_dir.resolve())}, resume_mapping=True)
    elif action == "resume":
        if not app.runtime.activate_continued_mapping():
            raise RequestError(503, "切换建图模式失败，请检查节点状态后重试")
    return app.runtime.snapshot()
