"""Revisioned navigation speed edits preserving the surrounding YAML text."""

import math

import yaml

from web_support import RequestError


SPEED_LIMITS = {
    "linear_speed_mps": (0.05, 2.0),
    "angular_speed_degps": (1.0, math.degrees(0.8)),
    "drive_angular_speed_degps": (1.0, math.degrees(0.8)),
}


def _speed_node(content, parameter):
    node = yaml.compose(content, Loader=yaml.SafeLoader)
    for key in ("nav_control", "ros__parameters", parameter):
        if not isinstance(node, yaml.MappingNode):
            raise RequestError(422, "nav.yaml 导航速度配置结构无效")
        matches = [value for name, value in node.value if name.value == key]
        if len(matches) != 1:
            raise RequestError(422, f"nav.yaml 中 {key} 缺失或重复")
        node = matches[0]
    if not isinstance(node, yaml.ScalarNode) or node.tag not in (
        "tag:yaml.org,2002:float", "tag:yaml.org,2002:int"
    ):
        raise RequestError(422, "导航速度必须是数值")
    return node


def read_navigation_speed(store, path):
    snapshot = store.read(path)
    values = {key: float(_speed_node(snapshot["content"], key).value) for key in SPEED_LIMITS}
    if not all(math.isfinite(value) for value in values.values()):
        raise RequestError(422, "导航速度必须是有限数值")
    return {**values, "revision": snapshot["revision"], "limits": SPEED_LIMITS}


def save_navigation_speed(store, path, body):
    for key, (minimum, maximum) in SPEED_LIMITS.items():
        value = body.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
            raise RequestError(400, f"{key} 范围为 {minimum}–{maximum}")
    snapshot = store.read(path)
    if body.get("expected_revision") != snapshot["revision"]:
        raise RequestError(409, "nav.yaml 已变化，请重新加载速度设置")
    content = snapshot["content"]
    replacements = [(_speed_node(content, key), float(body[key])) for key in SPEED_LIMITS]
    updated = content
    for node, value in sorted(replacements, key=lambda item: item[0].start_mark.index, reverse=True):
        updated = updated[:node.start_mark.index] + str(value) + updated[node.end_mark.index:]
    revision = store.save(path, {"content": updated, "expected_revision": snapshot["revision"]})
    return {**{key: float(body[key]) for key in SPEED_LIMITS}, "revision": revision, "saved": True}
