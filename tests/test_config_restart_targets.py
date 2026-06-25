import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _assignment_value(module, name):
    for node in module.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
                return ast.literal_eval(node.value)
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.target.id == name:
                return ast.literal_eval(node.value)
    raise AssertionError(f"assignment not found: {name}")


def test_config_restart_targets_do_not_restart_whole_base_drive_stack():
    source = (ROOT / "server" / "server_app.py").read_text(encoding="utf-8")
    module = ast.parse(source)
    targets = _assignment_value(module, "RESTART_TARGETS")
    impacts = _assignment_value(module, "CONFIG_IMPACTS")

    assert impacts["base_control.yaml"]["restart_targets"] == ["base"]
    assert impacts["joy.yaml"]["restart_targets"] == ["joy"]
    assert targets["base"] == "底盘驱动"
    assert targets["joy"] == "摇杆控制"
    assert "base_drive" not in targets
