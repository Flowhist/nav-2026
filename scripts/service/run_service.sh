#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/ros_environment.sh"
case "${1:-}" in
    perception)
        source "${FINAV_DCCS_SETUP:-/home/embotic/DCCS/install/local_setup.bash}"
        source "$FINAV_WORKSPACE_DIR/install/local_setup.bash"
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1
        ;;
    web|mapping|navigation|lidar) ;;
    *) printf 'Expected web, mapping, navigation, perception or lidar; base is owned by base_control\n' >&2; exit 2 ;;
esac
cd "$FINAV_REPO_DIR"
exec python3 "$FINAV_REPO_DIR/scripts/service/service_runner.py" "$1" \
    >> "$FINAV_STATE_DIR/$1.log" 2>&1
