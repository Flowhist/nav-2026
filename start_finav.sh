#!/usr/bin/env bash
# start_finav.sh
# 启动底盘驱动、摇杆控制、网页后台与本地键盘路由
#
# 用法：
#   bash start_finav.sh
#   bash start_finav.sh --joy-port /dev/ttyUSB1 --host 0.0.0.0 --port 8010

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR"
WORKSPACE_DIR="$(cd "$REPO_DIR/../.." && pwd)"

JOY_PORT="/dev/ttyUSB0"
SERVER_HOST="0.0.0.0"
SERVER_PORT="8010"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --joy-port)
            JOY_PORT="$2"
            shift 2
            ;;
        --host)
            SERVER_HOST="$2"
            shift 2
            ;;
        --port)
            SERVER_PORT="$2"
            shift 2
            ;;
        *)
            printf '未知参数: %s\n' "$1"
            exit 1
            ;;
    esac
done

DRIVER_PID=""
JOY_PID=""
SERVER_PID=""

cleanup() {
    printf '\n\n\n正在停止节点...\n'
    [[ -n "$SERVER_PID" ]] && kill "$SERVER_PID" 2>/dev/null || true
    [[ -n "$JOY_PID"    ]] && kill "$JOY_PID"    2>/dev/null || true
    [[ -n "$DRIVER_PID" ]] && kill "$DRIVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" "$JOY_PID" "$DRIVER_PID" 2>/dev/null || true
    printf '已停止。\n'
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WORKSPACE_DIR/install/local_setup.bash" 2>/dev/null || source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true

export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO_DIR/config/fastdds_profiles.xml"
export FINAV_REPO_DIR="$REPO_DIR"
export FINAV_MAPS_DIR="$REPO_DIR/maps"

printf '清理旧进程...\n'
pkill -9 -f "base_control.py" 2>/dev/null || true
pkill -9 -f "joystick_control.py" 2>/dev/null || true
pkill -9 -f "js_kb_router.py" 2>/dev/null || true
pkill -9 -f "server/run_server.py" 2>/dev/null || true
sleep 1

CONTROL_PARAMS="$(ros2 pkg prefix finav)/share/finav/config/base_control.yaml"

printf '▶ 启动 base_control\n'
ros2 run finav base_control.py \
    --ros-args --params-file "$CONTROL_PARAMS" \
    > /dev/null 2>&1 &
DRIVER_PID=$!

printf '▶ 启动 joystick_control\n'
ros2 run finav joystick_control.py \
    --ros-args --params-file "$CONTROL_PARAMS" -p "joystick_port:=$JOY_PORT" \
    > /dev/null 2>&1 &
JOY_PID=$!

printf '▶ 启动 web server\n'
python3 "$REPO_DIR/server/run_server.py" --host "$SERVER_HOST" --port "$SERVER_PORT" &
SERVER_PID=$!

printf '等待节点就绪...\n'
sleep 2.5

if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
    printf '\033[31m✗ base_control 启动失败\n'
    exit 1
fi
if ! kill -0 "$JOY_PID" 2>/dev/null; then
    printf '\033[31m✗ joystick_control 启动失败\n'
    exit 1
fi
if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    printf '\033[31m✗ web server 启动失败\n'
    exit 1
fi

printf '\033[32m✓ 底盘、摇杆与 Web 后台均已启动\033[0m\n'
printf '  摇杆串口: %s\n' "$JOY_PORT"
printf '  Web 地址: http://%s:%s\n' "$SERVER_HOST" "$SERVER_PORT"
printf '  F=键盘开关  │  Ctrl-C 退出\n\n'

ros2 run finav js_kb_router.py \
    --ros-args --params-file "$CONTROL_PARAMS" || true
