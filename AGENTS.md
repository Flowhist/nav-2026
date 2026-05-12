# Project Guide for Codex

## Project Overview
- Finav is a ROS 2 Humble navigation project for a WHILL-based robot.
- It includes chassis control, FREE lidar, DM-IMU, EKF fusion, SLAM Toolbox mapping/localization, custom planning/control, a web debug backend, and Gazebo/RViz simulation.
- Read `Doc/项目Wiki.md` before deep subsystem work.

## Tech Stack
- ROS 2 Humble with `ament_cmake`
- Python ROS nodes via `rclpy`
- C++14 ROS node for FREE lidar via `rclcpp`
- SLAM Toolbox, robot_localization, Nav2 map server components
- Gazebo / ros_gz_bridge simulation and RViz visualization
- Python HTTP backend with static HTML/CSS/JS frontend
- NumPy and OpenCV for lightweight localization utilities

## Common Commands
- Install: Not confirmed.
- Run:
  - `bash start_finav.sh`
  - `ros2 launch finav map.launch.py`
  - `ros2 launch finav nav.launch.py`
  - `python3 server/run_server.py --host 0.0.0.0 --port 8010`
- Test:
  - `colcon test --base-paths src/finav --packages-select finav`
- Lint: Not confirmed.
- Build:
  - `source /opt/ros/humble/setup.bash`
  - `colcon build --base-paths src/finav --packages-select finav --cmake-clean-cache`
  - `source install/setup.bash`

## Project Structure
- `launch/`: real-robot mapping/navigation and component launch files.
- `config/`: lidar, IMU, EKF, SLAM, chassis, planner, controller, and localization parameters.
- `scripts/control/`: chassis control, teleop routing, path planning, and path following.
- `scripts/imu/`: DM-IMU ROS publisher and configuration helper.
- `scripts/localization/`: lightweight localization helpers.
- `scripts/tool/`: map save, cleanup, TF, and operational scripts.
- `server/`: web debug backend; `server/web/` contains frontend assets.
- `sim/`: Gazebo/RViz simulation launch files, scripts, models, and worlds.
- `third_party/`: vendored FREE lidar and DM-IMU protocol code.
- `urdf/`, `rviz/`, `maps/`, `Doc/`: robot model, visualization configs, saved maps, and documentation.

## Working Rules
- Before large changes, inspect relevant files and present a short plan.
- Prefer minimal, localized edits.
- Preserve existing architecture and naming conventions.
- After edits, run the smallest relevant verification command.
- Do not delete files or rewrite large modules unless explicitly requested.
- Do not read `.env`, credential, token, private key, or secret files.
- Prefer scoped builds because this workspace may contain duplicate `finav` packages.

## Context7 Rule
- When working with third-party libraries, frameworks, APIs, SDKs, configuration formats, or version-specific behavior, use Context7 to fetch current documentation before implementing or explaining.

## Serena Rule
- For codebase exploration, refactoring, finding definitions/references, or understanding call chains, prefer Serena's symbolic tools before broad text search.
- Use Serena to locate relevant functions/classes/modules before editing.

## Git Rules
- Do not run `git push`.
- Do not create commits unless explicitly requested.
- When asked for a commit message, inspect the current diff first.

## Notes for Future Sessions
- Keep the external lidar contract as a single `/scan` topic unless explicitly asked to change downstream interfaces.
- Real-robot navigation uses SLAM Toolbox localization, custom `path_plan.py`, and `nav_control.py`.
- For ROS launch checks blocked by log permissions, set `ROS_LOG_DIR=/tmp/ros_log`.
