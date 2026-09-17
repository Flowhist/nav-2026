#!/usr/bin/env bash
# Compatibility entry point; the base implementation belongs to base_control.
set -eo pipefail
NAV_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$NAV_REPO/scripts/service/legacy_guard.sh"
source /opt/ros/humble/setup.bash
source "$NAV_REPO/../../install/local_setup.bash"
export BASE_CONTROL_SETUP="$NAV_REPO/../../install/local_setup.bash"
exec bash "$(ros2 pkg prefix base_control)/share/base_control/scripts/base_drive.sh" "$@"
