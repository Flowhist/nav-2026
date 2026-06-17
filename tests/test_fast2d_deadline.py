import importlib.util
import time
from pathlib import Path


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class BlockingGridPlanner:
    map_w = 140
    map_h = 140
    max_search_expansions = 1_000_000
    max_planning_time_s = 0.0
    plan_stats = {"expansions": 0}

    def _is_grid_free(self, idx):
        x, y = idx
        return 0 <= x < self.map_w and 0 <= y < self.map_h


def test_fast2d_grid_astar_respects_planning_deadline():
    module = _load_module(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "path_planning" / "fast2d.py",
        "fast2d_deadline_under_test",
    )
    planner = BlockingGridPlanner()

    start = time.monotonic()
    path = module.grid_astar(planner, (0, 0), (139, 139))
    elapsed = time.monotonic() - start

    assert path == []
    assert elapsed < 0.2
