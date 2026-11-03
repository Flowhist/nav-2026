#!/usr/bin/env bash
set -euo pipefail
venv=${FINAV_VISION_ENV:-/home/embotic/nav_workspace/.venv-vision}
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ -d "$here/../vision" ]]; then
  modules=$(cd "$here/.." && pwd)
else
  modules=$here
fi
[[ -x "$venv/bin/python" ]] || { echo "Run scripts/vision/setup.sh first" >&2; exit 1; }
# The ROS overlay is inherited from ros2 launch. Put our SDK and NumPy ahead of DCCS.
site=$("$venv/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
export PYTHONPATH="$site:$modules${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$site/pyorbbecsdk${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONNOUSERSITE=1
case ${1:-} in
  camera) module=vision.camera ;;
  tracker) module=vision.tracker_node ;;
  prepare) module=vision.prepare_models ;;
  *) echo 'Usage: run_vision.sh camera|tracker|prepare [arguments]' >&2; exit 2 ;;
esac
shift
exec "$venv/bin/python" -m "$module" "$@"
