import ast
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server"))


def _assignment_value(module, name):
    for node in module.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
                return ast.literal_eval(node.value)
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.target.id == name:
                return ast.literal_eval(node.value)
    raise AssertionError(f"assignment not found: {name}")


def test_config_restart_targets_restart_control_stack_together():
    source = (ROOT / "server" / "server_app.py").read_text(encoding="utf-8")
    module = ast.parse(source)
    targets = _assignment_value(module, "RESTART_TARGETS")
    impacts = _assignment_value(module, "CONFIG_IMPACTS")

    assert impacts["base_control.yaml"]["restart_targets"] == ["base_drive"]
    assert impacts["joy.yaml"]["restart_targets"] == ["base_drive"]
    assert targets == {"base_drive": "底盘控制"}


def test_supervisor_path_matches_relative_command_line():
    source = (ROOT / "server" / "process_manager.py").read_text(encoding="utf-8")
    module = ast.parse(source)
    function_names = {
        node.name for node in module.body if isinstance(node, ast.FunctionDef)
    }

    assert "_cmdline_references_script" in function_names

    namespace = {}
    exec(compile(module, "process_manager.py", "exec"), namespace)
    matches = namespace["_cmdline_references_script"]

    assert matches(
        ["bash", "./src/finav/start_finav.sh"],
        Path("/home/embotic/nav_workspace"),
        Path("/home/embotic/nav_workspace/src/finav/start_finav.sh"),
    )


def test_control_stack_uses_process_groups_for_restart():
    script = (ROOT / "start_finav.sh").read_text(encoding="utf-8")

    assert "force_stop_group" in script
    assert script.count("setsid ros2") == 3
