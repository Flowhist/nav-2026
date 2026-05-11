# Repository Guidelines

## Project Structure & Module Organization
`finav` is a ROS 2 Humble `ament_cmake` package for real-robot navigation and simulation. Key directories:

- `launch/` and `launch/sub/`: real-robot mapping, navigation, and component launch files.
- `config/`: lidar, IMU, EKF, SLAM Toolbox, chassis, planner, and controller YAMLs.
- `scripts/control/`: Python ROS nodes for chassis control, teleop routing, planning, and path following.
- `scripts/imu/`: DM-IMU ROS wrapper and local configuration helper.
- `third_party/free_lidar/` and `third_party/dm_imu/`: vendored device/protocol code.
- `server/`: Python HTTP backend plus `server/web/` static JS/CSS UI.
- `sim/`: Gazebo/RViz simulation launch files, scripts, worlds, and configs.
- `urdf/`, `rviz/`, `maps/`, and `Doc/`: robot model, visualization configs, saved maps, and project documentation.

## Build, Test, and Development Commands
Run from the workspace root unless noted.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/finav --packages-select finav --cmake-clean-cache
source install/setup.bash
```

Use the scoped build when another `finav` package exists in the workspace. For checks:

```bash
colcon test --base-paths src/finav --packages-select finav
ros2 launch finav map.launch.py
ros2 launch finav nav.launch.py
bash src/finav/start_finav.sh
```

If ROS logging is blocked by permissions, set `ROS_LOG_DIR=/tmp/ros_log`.

## Coding Style & Naming Conventions
Follow existing names and module boundaries. Python files use 4-space indentation, `snake_case` functions/variables, and executable node scripts under `scripts/`. C++ uses C++14 and compiles with `-Wall -Wextra -Wpedantic`; keep vendor-facing lidar code under `third_party/free_lidar/`. Launch files use `*.launch.py`; YAML config names should match the subsystem they tune.

## Testing Guidelines
There is no dedicated `tests/` directory yet. `BUILD_TESTING` enables `ament_lint_auto`, so run `colcon test` after build-impacting changes. For runtime changes, verify the smallest affected launch path and inspect relevant ROS topics, for example `/scan`, `/cmd_vel`, `/odom`, `/tf`, or `/plan`.

## Commit & Pull Request Guidelines
Recent commits use short, focused Chinese summaries such as `雷达频率恒定30hz（否则畸变）` or `更新硬件参数`. Keep commits concise and scoped to one behavior. PRs should include the affected subsystem, commands run, hardware/simulation context, and screenshots or logs for UI, RViz, or robot-behavior changes.

## Agent-Specific Instructions
Prefer minimal localized edits. Preserve the external `/scan` lidar contract unless a task explicitly changes downstream interfaces. Do not read secret files. Do not delete maps, logs, or generated artifacts without explicit approval.
