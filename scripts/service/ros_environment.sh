#!/usr/bin/env bash
# Shared environment for service entry points; source from bash.
set -eo pipefail
FINAV_REPO_DIR="${FINAV_REPO_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
FINAV_WORKSPACE_DIR="$(cd "$FINAV_REPO_DIR/../.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$FINAV_WORKSPACE_DIR/install/local_setup.bash" ]]; then
    source "$FINAV_WORKSPACE_DIR/install/local_setup.bash"
else
    source "$FINAV_WORKSPACE_DIR/install/setup.bash"
fi
export FINAV_REPO_DIR
export FINAV_MAPS_DIR="$FINAV_REPO_DIR/maps"
export FINAV_STATE_DIR="${FINAV_STATE_DIR:-$HOME/.local/state/finav}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FINAV_REPO_DIR/config/fastdds_profiles.xml"
export LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONIOENCODING=utf-8 PYTHONUNBUFFERED=1
export ROS_LOG_DIR="$FINAV_STATE_DIR/ros_log"
mkdir -p "$ROS_LOG_DIR"
