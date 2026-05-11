#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"

MAP_FILE="${1:-map4}"
MAPS_DIR="${FINAV_MAPS_DIR:-${REPO_DIR}/maps}"

source /opt/ros/humble/setup.bash
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"

echo "[auto_localize_sim] map_file=${MAP_FILE}"
echo "[auto_localize_sim] maps_dir=${MAPS_DIR}"

ros2 run finav auto_localize.py \
  --ros-args \
  -p use_sim_time:=true \
  -p maps_dir:="${MAPS_DIR}" \
  -p map_file:="${MAP_FILE}" \
  -p scan_topic:=/scan \
  -p initialpose_topic:=/initialpose
