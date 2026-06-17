"""Optional SE2 planner helpers used by nav_path_plan."""

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

from path_planning.fast2d import build_grid_corridor, grid_astar, state_allowed_by_corridor

GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]
WorldPose = Tuple[float, float, float]
State3D = Tuple[int, int, int]


def build_footprint_samples(planner) -> None:
    dx_step = max(0.04, planner.footprint_sample_step_m)
    dy_step = max(0.04, planner.footprint_sample_step_m)
    x_min = -planner.vehicle_rear_m - planner.vehicle_margin_m
    x_max = planner.vehicle_front_m + planner.vehicle_margin_m
    y_min = -planner.vehicle_right_m - planner.vehicle_margin_m
    y_max = planner.vehicle_left_m + planner.vehicle_margin_m
    xs = planner._sample_axis(x_min, x_max, dx_step)
    ys = planner._sample_axis(y_min, y_max, dy_step)
    planner.footprint_samples = [(x, y) for x in xs for y in ys]


def pose_is_free(planner, pose: WorldPose) -> bool:
    planner.plan_stats["pose_checks"] += 1
    px, py, yaw = pose
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    for fx, fy in planner.footprint_samples:
        wx = px + fx * cy - fy * sy
        wy = py + fx * sy + fy * cy
        idx = planner._world_to_grid((wx, wy))
        if idx is None or not planner._is_grid_free(idx):
            return False
    return True


def state_is_free(planner, state: State3D) -> bool:
    cached = planner.pose_free_cache.get(state)
    if cached is not None:
        planner.plan_stats["state_cache_hits"] += 1
        return cached
    planner.plan_stats["state_cache_misses"] += 1
    free = pose_is_free(planner, planner._state_to_pose(state))
    planner.pose_free_cache[state] = free
    return free


def ordered_heading_bins(planner, center_yaw: float, limit: Optional[int] = None) -> List[int]:
    center = planner._yaw_to_bin(center_yaw)
    ordered: List[int] = [center]
    max_count = planner.heading_bins if limit is None else max(1, limit)
    for delta in range(1, planner.heading_bins):
        ordered.append((center + delta) % planner.heading_bins)
        if len(ordered) >= max_count:
            break
        ordered.append((center - delta) % planner.heading_bins)
        if len(ordered) >= max_count:
            break
    return ordered


def merge_heading_candidates(planner, primary_yaw: float, secondary_yaw: float, limit: int) -> List[int]:
    merged: List[int] = []
    seen = set()
    for yaw in (primary_yaw, secondary_yaw):
        for heading_bin in ordered_heading_bins(planner, yaw):
            if heading_bin in seen:
                continue
            seen.add(heading_bin)
            merged.append(heading_bin)
            if len(merged) >= limit:
                return merged
    return merged


def nearest_free_pose(planner, src_pose: WorldPose, heading_candidates: List[int], max_radius_cells: int = 40) -> Optional[WorldPose]:
    base_idx = planner._world_to_grid((src_pose[0], src_pose[1]))
    if base_idx is None:
        return None
    for radius in range(0, max_radius_cells + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if radius > 0 and max(abs(dx), abs(dy)) != radius:
                    continue
                gx = base_idx[0] + dx
                gy = base_idx[1] + dy
                if not planner._is_grid_free((gx, gy)):
                    continue
                for h in heading_candidates:
                    state = (gx, gy, h)
                    if state_is_free(planner, state):
                        return planner._state_to_pose(state)
    return None


def nearest_reachable_pose(planner, src_pose: WorldPose, heading_candidates: List[int], max_radius_cells: int = 40) -> Optional[WorldPose]:
    base_idx = planner._world_to_grid((src_pose[0], src_pose[1]))
    if base_idx is None:
        return None
    for radius in range(0, max_radius_cells + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if radius > 0 and max(abs(dx), abs(dy)) != radius:
                    continue
                gx = base_idx[0] + dx
                gy = base_idx[1] + dy
                if not planner._is_grid_free((gx, gy)):
                    continue
                for h in heading_candidates:
                    state = (gx, gy, h)
                    if not state_is_free(planner, state):
                        continue
                    pose = planner._state_to_pose(state)
                    if not motion_segment_is_free(planner, src_pose, pose):
                        continue
                    return pose
    return None


def build_transition_path(planner, current_pose: WorldPose, start_pose: WorldPose) -> List[WorldPose]:
    dist = math.hypot(start_pose[0] - current_pose[0], start_pose[1] - current_pose[1])
    yaw_delta = abs(planner._norm_angle(start_pose[2] - current_pose[2]))
    if dist < 1e-3 and yaw_delta < math.radians(2.0):
        return []
    if not motion_segment_is_free(planner, current_pose, start_pose):
        return []
    return [current_pose, start_pose]


def motion_segment_is_free(planner, a: WorldPose, b: WorldPose) -> bool:
    planner.plan_stats["motion_checks"] += 1
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    yaw_delta = abs(planner._norm_angle(b[2] - a[2]))
    yaw_arc = max(planner.vehicle_front_m + planner.vehicle_rear_m, 0.2) * yaw_delta
    span = max(dist, yaw_arc)
    step = max(planner.resolution * 0.5, planner.footprint_sample_step_m * 0.75, 0.04)
    count = max(2, int(math.ceil(span / step)))
    planner.plan_stats["motion_samples"] += count + 1
    for i in range(count + 1):
        t = i / count
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        yaw = interp_angle(a[2], b[2], t)
        if not pose_is_free(planner, (x, y, yaw)):
            return False
    return True


def interp_angle(a: float, b: float, t: float) -> float:
    delta = math.atan2(math.sin(b - a), math.cos(b - a))
    return math.atan2(math.sin(a + delta * t), math.cos(a + delta * t))


def heuristic(planner, a: State3D, goal_idx: GridIndex, goal_pose: WorldPose) -> float:
    wx, wy = planner._grid_to_world((a[0], a[1]))
    gx, gy = planner._grid_to_world(goal_idx)
    dist = math.hypot(wx - gx, wy - gy)
    if planner.heuristic_heading_weight <= 1e-9:
        return dist
    desired_yaw = math.atan2(goal_pose[1] - wy, goal_pose[0] - wx)
    state_yaw = planner._bin_to_yaw(a[2])
    yaw_err = abs(planner._norm_angle(desired_yaw - state_yaw))
    return dist + planner.heuristic_heading_weight * yaw_err


def expand_state(planner, state: State3D) -> List[Tuple[State3D, float]]:
    pose = planner._state_to_pose(state)
    out: List[Tuple[State3D, float]] = []
    if planner.enable_turn_in_place and planner.primitive_turn_bins > 0:
        for turn_bins in (-planner.primitive_turn_bins, planner.primitive_turn_bins):
            new_heading_bin = (state[2] + turn_bins) % planner.heading_bins
            nb_state = (state[0], state[1], new_heading_bin)
            if state_is_free(planner, nb_state) and motion_segment_is_free(planner, pose, planner._state_to_pose(nb_state)):
                turn_cost = planner.turn_cost_weight * abs(turn_bins)
                out.append((nb_state, planner.turn_in_place_cost + turn_cost))
    for turn_bins in planner.turn_options:
        new_heading_bin = (state[2] + turn_bins) % planner.heading_bins
        new_yaw = planner._bin_to_yaw(new_heading_bin)
        avg_yaw = interp_angle(pose[2], new_yaw, 0.5)
        nx = pose[0] + planner.primitive_step_m * math.cos(avg_yaw)
        ny = pose[1] + planner.primitive_step_m * math.sin(avg_yaw)
        nb_idx = planner._world_to_grid((nx, ny))
        if nb_idx is None:
            continue
        wx, wy = planner._grid_to_world(nb_idx)
        nb_pose = (wx, wy, new_yaw)
        nb_state = (nb_idx[0], nb_idx[1], new_heading_bin)
        if not state_is_free(planner, nb_state):
            continue
        if not motion_segment_is_free(planner, pose, nb_pose):
            continue
        turn_cost = planner.turn_cost_weight * abs(turn_bins)
        out.append((nb_state, planner.primitive_step_m + turn_cost))
    planner.plan_stats["neighbors"] += len(out)
    return out


def build_search_corridor(planner, start_pose: WorldPose, goal_pose: WorldPose) -> Optional[Set[GridIndex]]:
    if not planner.enable_search_corridor or planner.search_corridor_radius_m <= 0.0:
        return None
    start_idx = planner._world_to_grid((start_pose[0], start_pose[1]))
    goal_idx = planner._world_to_grid((goal_pose[0], goal_pose[1]))
    if start_idx is None or goal_idx is None:
        return None
    coarse = grid_astar(planner, start_idx, goal_idx)
    if not coarse:
        return None
    radius_cells = max(1, int(math.ceil(planner.search_corridor_radius_m / max(planner.resolution, 1e-6))))
    return build_grid_corridor(coarse, radius_cells, planner.map_w, planner.map_h)


def astar(planner, start_pose: WorldPose, goal_pose: WorldPose, corridor: Optional[Set[GridIndex]] = None) -> List[WorldPose]:
    start_state = planner._pose_to_state(start_pose)
    planner.active_search_corridor = corridor
    goal_idx = planner._world_to_grid((goal_pose[0], goal_pose[1]))
    if start_state is None or goal_idx is None:
        return []
    open_heap: List[Tuple[float, float, State3D]] = []
    start_h = heuristic(planner, start_state, goal_idx, goal_pose)
    heapq.heappush(open_heap, (planner.heuristic_weight * start_h, 0.0, start_state))
    g_cost: Dict[State3D, float] = {start_state: 0.0}
    parent: Dict[State3D, State3D] = {}
    closed = set()
    expansions = 0
    goal_radius_cells = max(1, int(math.ceil(planner.goal_tolerance_m / max(planner.resolution, 1e-6))))
    reached: Optional[State3D] = None
    deadline = __import__('time').monotonic() + planner.max_planning_time_s
    while open_heap:
        if __import__('time').monotonic() > deadline:
            return []
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expansions += 1
        planner.plan_stats["expansions"] = expansions
        if expansions > planner.max_search_expansions:
            return []
        if max(abs(current[0] - goal_idx[0]), abs(current[1] - goal_idx[1])) <= goal_radius_cells:
            reached = current
            break
        for nb, step_cost in expand_state(planner, current):
            if nb in closed:
                continue
            if not state_allowed_by_corridor(nb, planner.active_search_corridor):
                continue
            ng = g + step_cost
            if ng < g_cost.get(nb, float("inf")):
                g_cost[nb] = ng
                parent[nb] = current
                f = ng + planner.heuristic_weight * heuristic(planner, nb, goal_idx, goal_pose)
                heapq.heappush(open_heap, (f, ng, nb))
    if reached is None:
        return []
    states = reconstruct_path(parent, reached)
    poses = [planner._state_to_pose(s) for s in states]
    if poses:
        terminal = poses[-1]
        goal_heading = goal_pose[2]
        approach_yaw = (
            math.atan2(goal_pose[1] - terminal[1], goal_pose[0] - terminal[0])
            if math.hypot(goal_pose[0] - terminal[0], goal_pose[1] - terminal[1]) > 1e-3
            else goal_heading
        )
        aligned_goal = (goal_pose[0], goal_pose[1], approach_yaw)
        if motion_segment_is_free(planner, terminal, aligned_goal):
            poses.append(aligned_goal)
        else:
            poses[-1] = (terminal[0], terminal[1], approach_yaw)
    return planner._annotate_path_yaw(poses, planner._final_goal_yaw(goal_pose[2]))


def reconstruct_path(parent: Dict[State3D, State3D], end: State3D) -> List[State3D]:
    path = [end]
    cur = end
    while cur in parent:
        cur = parent[cur]
        path.append(cur)
    path.reverse()
    return path
