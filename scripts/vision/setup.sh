#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
venv=${FINAV_VISION_ENV:-/home/embotic/nav_workspace/.venv-vision}
uv=${UV_BIN:-$HOME/.local/bin/uv}
"$uv" venv --python /usr/bin/python3 --system-site-packages --allow-existing "$venv"
"$uv" pip install --python "$venv/bin/python" -r "$root/requirements/vision.txt" -c "$root/requirements/vision-constraints.txt"
# SDK's declared OpenCV constraint conflicts with the tested NumPy 1.x ROS ABI.
"$uv" pip install --python "$venv/bin/python" --no-deps pyorbbecsdk2==2.1.2
if ! "$venv/bin/python" -c 'import tensorrt; assert tensorrt.__version__ == "10.3.0"' 2>/dev/null; then
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT
  (cd "$tmp" && apt-get download 'python3-libnvinfer=10.3.0.30-1+cuda12.5')
  dpkg-deb -x "$tmp"/*.deb "$tmp/unpacked"
  site=$("$venv/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
  cp -a "$tmp/unpacked/usr/lib/python3.10/dist-packages/"* "$site/"
fi
site=$("$venv/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
PYTHONPATH="$site${PYTHONPATH:+:$PYTHONPATH}" LD_LIBRARY_PATH="$site/pyorbbecsdk${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$venv/bin/python" -c 'import tensorrt, torch, cv2, pyorbbecsdk; print("TensorRT", tensorrt.__version__, "torch", torch.__version__, "OpenCV", cv2.__version__, "SDK", pyorbbecsdk.__file__)'
"$uv" pip freeze --python "$venv/bin/python" > "$venv/requirements-resolved.txt"
echo "Environment ready: $venv"
