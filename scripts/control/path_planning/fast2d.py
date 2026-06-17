"""Fast 2D grid planner used by nav_path_plan."""

import heapq
import math
import time
from typing import Dict, List, Optional, Set, Tuple

GridIndex = Tuple[int, int]
WorldPose = Tuple[float, float, float]
State3D = Tuple[int, int, int]


def build_grid_corridor(
    coarse_path: List[GridIndex], radius_cells: int, width: int, height: int
) -> Set[GridIndex]:
    """Build a grid-cell corridor around a coarse 2D path."""
    corridor: Set[GridIndex] = set()
    if not coarse_path:
        return corridor
    radius = max(0, int(radius_cells))
    samples: List[GridIndex] = []
    for i in range(len(coarse_path) - 1):
        ax, ay = coarse_path[i]
        bx, by = coarse_path[i + 1]
        steps = max(abs(bx - ax), abs(by - ay), 1)
        for j in range(steps + 1):
            t = j / float(steps)
            samples.append((round(ax + (bx - ax) * t), round(ay + (by - ay) * t)))
    if len(coarse_path) == 1:
        samples.append(coarse_path[0])
    for cx, cy in samples:
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                x = cx + dx
                y = cy + dy
                if 0 <= x < width and 0 <= y < height:
                    corridor.add((x, y))
    return corridor


def state_allowed_by_corridor(state: State3D, corridor: Optional[Set[GridIndex]]) -> bool:
    return corridor is None or (state[0], state[1]) in corridor


def plan_fast2d(planner, current_pose: WorldPose, goal_pose: WorldPose) -> List[WorldPose]:
    """Plan quickly in 2D grid space and annotate yaw from path tangent."""
    start_idx = planner._world_to_grid((current_pose[0], current_pose[1]))
    goal_idx = planner._world_to_grid((goal_pose[0], goal_pose[1]))
    if start_idx is None or goal_idx is None:
        return []
    if not planner._is_grid_free(start_idx) or not planner._is_grid_free(goal_idx):
        return []

    coarse = grid_astar(planner, start_idx, goal_idx)
    if not coarse:
        return []
    poses: List[WorldPose] = []
    for idx in coarse:
        x, y = planner._grid_to_world(idx)
        poses.append((x, y, 0.0))
    if poses:
        poses[0] = (current_pose[0], current_pose[1], current_pose[2])
        poses[-1] = (goal_pose[0], goal_pose[1], goal_pose[2])
    poses = shortcut_path_2d(planner, poses)
    poses = planner._densify_path(poses)
    return planner._annotate_path_yaw(poses, planner._final_goal_yaw(goal_pose[2]))


def shortcut_path_2d(planner, path: List[WorldPose]) -> List[WorldPose]:
    """Shortcut a 2D path using the inflated occupancy grid only."""
    if len(path) <= 2:
        return path
    simplified: List[WorldPose] = [path[0]]
    i = 0
    while i < len(path) - 1:
        picked = i + 1
        for j in range(len(path) - 1, i, -1):
            if grid_segment_is_free(planner, path[i], path[j]):
                picked = j
                break
        simplified.append(path[picked])
        i = picked
    return planner._annotate_path_yaw(simplified, planner._final_goal_yaw(path[-1][2]))


def grid_segment_is_free(planner, a: WorldPose, b: WorldPose) -> bool:
    """Check a straight segment against the inflated occupancy grid."""
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    step = max(planner.resolution * 0.5, 0.02)
    count = max(1, int(math.ceil(dist / step)))
    for i in range(count + 1):
        t = i / count
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        idx = planner._world_to_grid((x, y))
        if idx is None or not planner._is_grid_free(idx):
            return False
    return True


def grid_astar(planner, start: GridIndex, goal: GridIndex) -> List[GridIndex]:
    if not planner._is_grid_free(start) or not planner._is_grid_free(goal):
        return []
    open_heap: List[Tuple[float, float, GridIndex]] = []
    heapq.heappush(open_heap, (0.0, 0.0, start))
    g_cost: Dict[GridIndex, float] = {start: 0.0}
    parent: Dict[GridIndex, GridIndex] = {}
    closed: Set[GridIndex] = set()
    neighbors = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    ]
    expansions = 0
    deadline = time.monotonic() + float(getattr(planner, "max_planning_time_s", 1.5))
    while open_heap:
        if time.monotonic() > deadline:
            return []
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return reconstruct_grid_path(parent, current)
        closed.add(current)
        expansions += 1
        if hasattr(planner, "plan_stats"):
            planner.plan_stats["expansions"] = expansions
        if expansions > getattr(planner, "search_corridor_max_expansions", 200000):
            return []
        for dx, dy, cost in neighbors:
            nb = (current[0] + dx, current[1] + dy)
            if nb in closed or not planner._is_grid_free(nb):
                continue
            ng = g + cost
            if ng < g_cost.get(nb, float("inf")):
                g_cost[nb] = ng
                parent[nb] = current
                h = math.hypot(nb[0] - goal[0], nb[1] - goal[1])
                heapq.heappush(open_heap, (ng + h, ng, nb))
    return []


def reconstruct_grid_path(parent: Dict[GridIndex, GridIndex], end: GridIndex) -> List[GridIndex]:
    path = [end]
    cur = end
    while cur in parent:
        cur = parent[cur]
        path.append(cur)
    path.reverse()
    return path
