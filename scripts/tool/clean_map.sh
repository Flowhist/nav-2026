#!/usr/bin/env bash

set -euo pipefail

patterns=(
  "ros2 launch finav map.launch.py"
  "launch/map.launch.py"
  "sync_slam_toolbox_node"
  "slam_toolbox"
  "free_lidar_node"
  "r2000_node"
  "dm_imu_publisher.py"
  "dm_imu_node.py"
  "ekf_node"
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

remaining="$(pgrep -fa "map.launch.py|sync_slam_toolbox_node|slam_toolbox|free_lidar_node|r2000_node|dm_imu|ekf_node|robot_state_publisher|joint_state_publisher|static_transform_publisher" || true)"
if [[ -n "$remaining" ]]; then
  printf '⚠ 建图相关残留进程:\n%s\n' "$remaining"
else
  printf '✓ 建图相关进程已清理\n'
fi
