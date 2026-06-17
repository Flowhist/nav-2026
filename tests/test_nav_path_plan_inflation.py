import importlib.util
import sys
from types import SimpleNamespace
from pathlib import Path


def _load_module(path: Path, name: str):
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_build_inflated_grid_matches_circular_radius():
    module = _load_module(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "nav_path_plan.py",
        "nav_path_plan_inflation_under_test",
    )
    width = 7
    height = 7
    data = [0] * (width * height)
    data[3 * width + 3] = 100

    inflated = module.build_inflated_grid(
        data,
        width,
        height,
        occupied_threshold=65,
        treat_unknown_as_occupied=True,
        inflation_cells=2,
    )

    expected = []
    for y in range(height):
        for x in range(width):
            expected.append(1 if (x - 3) ** 2 + (y - 3) ** 2 <= 4 else 0)
    assert inflated == expected


def test_build_inflated_grid_handles_unknown_policy_and_zero_radius():
    module = _load_module(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "nav_path_plan.py",
        "nav_path_plan_inflation_unknown_under_test",
    )
    data = [0, -1, 80, 10]

    conservative = module.build_inflated_grid(data, 2, 2, 65, True, 0)
    permissive = module.build_inflated_grid(data, 2, 2, 65, False, 0)

    assert conservative == [0, 1, 1, 0]
    assert permissive == [0, 0, 1, 0]


def test_map_callback_defers_inflation_until_planning_needs_it():
    module = _load_module(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "nav_path_plan.py",
        "nav_path_plan_lazy_map_under_test",
    )
    planner = object.__new__(module.PathPlanner)
    planner.map_frame = "map"
    planner.map_seq = -1
    planner.map_msg = None
    planner.map_dirty = False
    planner.replan_on_map_update = False
    planner._map_processed_dirty = False
    planner.inflated_grid = []
    planner.pose_free_cache = {"stale": True}

    rebuild_calls = []

    def fake_rebuild():
        rebuild_calls.append(True)
        planner.inflated_grid = [0, 1, 0, 0]

    planner._rebuild_processed_map = fake_rebuild
    msg = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(sec=1, nanosec=2),
        ),
        info=SimpleNamespace(width=2, height=2),
        data=[0, 100, 0, 0],
    )

    module.PathPlanner._on_map(planner, msg)

    assert planner.map_msg is msg
    assert planner._map_processed_dirty is True
    assert rebuild_calls == []
    assert planner.pose_free_cache == {"stale": True}

    assert module.PathPlanner._ensure_processed_map(planner) is True
    assert rebuild_calls == [True]
    assert planner._map_processed_dirty is False


def test_map_update_retries_only_after_failed_goal_when_replan_on_map_update_disabled():
    module = _load_module(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "nav_path_plan.py",
        "nav_path_plan_failed_goal_retry_under_test",
    )
    planner = object.__new__(module.PathPlanner)
    planner.map_frame = "map"
    planner.map_seq = -1
    planner.map_msg = None
    planner.map_dirty = False
    planner.replan_on_map_update = False
    planner.plan_failed_for_current_goal = True
    planner.goal_pose_world = (1.0, 2.0, 0.0)
    planner._map_processed_dirty = False

    msg = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(sec=2, nanosec=3),
        ),
        info=SimpleNamespace(width=2, height=2),
        data=[0, 0, 0, 0],
    )

    module.PathPlanner._on_map(planner, msg)

    assert planner._map_processed_dirty is True
    assert planner.map_dirty is True


def test_map_update_does_not_replan_active_successful_path_when_disabled():
    module = _load_module(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "nav_path_plan.py",
        "nav_path_plan_no_active_map_replan_under_test",
    )
    planner = object.__new__(module.PathPlanner)
    planner.map_frame = "map"
    planner.map_seq = -1
    planner.map_msg = None
    planner.map_dirty = False
    planner.replan_on_map_update = False
    planner.plan_failed_for_current_goal = False
    planner.goal_pose_world = (1.0, 2.0, 0.0)
    planner._map_processed_dirty = False

    msg = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(sec=3, nanosec=4),
        ),
        info=SimpleNamespace(width=2, height=2),
        data=[0, 0, 0, 0],
    )

    module.PathPlanner._on_map(planner, msg)

    assert planner._map_processed_dirty is True
    assert planner.map_dirty is False
