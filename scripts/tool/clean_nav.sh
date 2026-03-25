#!/usr/bin/env bash

set -euo pipefail

patterns=(
  "ros2 launch finav nav.launch.py"
  "launch/nav.launch.py"
  "localization_slam_toolbox_node"
  "slam_toolbox"
  "r2000_node"
  "scan_angle_filter.py"
  "dm_imu_publisher.py"
  "dm_imu_node.py"
  "ekf_node"
  "path_plan.py"
  "nav_control.py"
  "robot_state_publisher"
  "joint_state_publisher"
  "static_transform_publisher"
)

for sig in TERM KILL; do
  for pattern in "${patterns[@]}"; do
    pkill "-$sig" -f "$pattern" 2>/dev/null || true
  done
  sleep 1
done

remaining="$(pgrep -fa "nav.launch.py|localization_slam_toolbox_node|slam_toolbox|r2000_node|scan_angle_filter.py|dm_imu|ekf_node|path_plan.py|nav_control.py|robot_state_publisher|joint_state_publisher|static_transform_publisher" || true)"
if [[ -n "$remaining" ]]; then
  printf '⚠ 导航相关残留进程:\n%s\n' "$remaining"
else
  printf '✓ 导航相关进程已清理\n'
fi
