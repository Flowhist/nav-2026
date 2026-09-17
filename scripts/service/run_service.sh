#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/ros_environment.sh"
case "${1:-}" in
    web|mapping|navigation) ;;
    *) printf 'Expected web, mapping or navigation; base is owned by base_control\n' >&2; exit 2 ;;
esac
cd "$FINAV_REPO_DIR"
exec python3 "$FINAV_REPO_DIR/scripts/service/service_runner.py" "$1" \
    >> "$FINAV_STATE_DIR/$1.log" 2>&1
