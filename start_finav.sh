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
ROUTER_PID=""
START_CHECKS=6
START_INTERVAL=0.1
CLEANING_UP=0
STOP_TIMEOUT=1.0
MAP_CLEAN_SCRIPT="$REPO_DIR/scripts/tool/clean_map.sh"
NAV_CLEAN_SCRIPT="$REPO_DIR/scripts/tool/clean_nav.sh"

fail() {
    printf '\033[31m%s\033[0m\n' "$1" >&2
    exit 1
}

wait_all_stable() {
    local i
    for ((i = 0; i < START_CHECKS; ++i)); do
        kill -0 "$DRIVER_PID" 2>/dev/null || fail "base_control 启动失败"
        kill -0 "$JOY_PID" 2>/dev/null || fail "joystick_control 启动失败"
        kill -0 "$SERVER_PID" 2>/dev/null || fail "web server 启动失败"
        sleep "$START_INTERVAL"
    done
}

force_stop_pid() {
    local pid="$1"
    [[ -n "$pid" ]] || return 0
    kill -TERM "$pid" 2>/dev/null || true
}

wait_pid_exit() {
    local pid="$1"
    local elapsed=0
    [[ -n "$pid" ]] || return 0
    while kill -0 "$pid" 2>/dev/null; do
        if awk "BEGIN { exit !($elapsed >= $STOP_TIMEOUT) }"; then
            kill -KILL "$pid" 2>/dev/null || true
            break
        fi
        sleep 0.1
        elapsed=$(awk "BEGIN { printf \"%.1f\", $elapsed + 0.1 }")
    done
}

cleanup() {
    [[ "$CLEANING_UP" -eq 1 ]] && return
    CLEANING_UP=1
    trap - EXIT INT TERM
    printf '\n\n\n正在停止节点...\n'
    force_stop_pid "$ROUTER_PID"
    force_stop_pid "$SERVER_PID"
    force_stop_pid "$JOY_PID"
    force_stop_pid "$DRIVER_PID"
    pkill -TERM -f "web_control_server" 2>/dev/null || true
    wait_pid_exit "$ROUTER_PID"
    wait_pid_exit "$SERVER_PID"
    wait_pid_exit "$JOY_PID"
    wait_pid_exit "$DRIVER_PID"
    pkill -KILL -f "web_control_server" 2>/dev/null || true
    wait "$ROUTER_PID" "$SERVER_PID" "$JOY_PID" "$DRIVER_PID" 2>/dev/null || true
    [[ -f "$MAP_CLEAN_SCRIPT" ]] && bash "$MAP_CLEAN_SCRIPT" >/dev/null 2>&1 || true
    [[ -f "$NAV_CLEAN_SCRIPT" ]] && bash "$NAV_CLEAN_SCRIPT" >/dev/null 2>&1 || true
    printf '已停止。\n'
}

handle_interrupt() {
    cleanup
    exit 130
}

trap cleanup EXIT
trap handle_interrupt INT TERM

ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP_LOCAL="$WORKSPACE_DIR/install/local_setup.bash"
WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"

[[ -f "$ROS_SETUP" ]] || fail "未找到 $ROS_SETUP，请先安装 ROS 2 Humble。"
# ROS setup 脚本内部会读取若干未定义变量，需临时关闭 nounset。
set +u
source "$ROS_SETUP"

if [[ -f "$WORKSPACE_SETUP_LOCAL" ]]; then
    source "$WORKSPACE_SETUP_LOCAL"
elif [[ -f "$WORKSPACE_SETUP" ]]; then
    source "$WORKSPACE_SETUP"
else
    set -u
    fail "未找到工作区 install/setup.bash。请先在 $WORKSPACE_DIR 执行 colcon build，再重新运行脚本。"
fi
set -u

command -v ros2 >/dev/null 2>&1 || fail "当前环境没有 ros2 命令，ROS 环境未正确加载。"

export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO_DIR/config/fastdds_profiles.xml"
export FINAV_REPO_DIR="$REPO_DIR"
export FINAV_MAPS_DIR="$REPO_DIR/maps"

printf '清理旧进程...\n'
pkill -9 -f "base_control.py" 2>/dev/null || true
pkill -9 -f "joystick_control.py" 2>/dev/null || true
pkill -9 -f "js_kb_router.py" 2>/dev/null || true
pkill -9 -f "server/run_server.py" 2>/dev/null || true
sleep 0.2

CONTROL_PARAMS="$REPO_DIR/config/base_control.yaml"
[[ -f "$CONTROL_PARAMS" ]] || fail "未找到参数文件: $CONTROL_PARAMS"

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

printf '等待关键进程稳定...\n'
wait_all_stable

printf '\033[32m✓ 底盘、摇杆与 Web 后台均已启动\033[0m\n'
printf '  摇杆串口: %s\n' "$JOY_PORT"
printf '  Web 地址: http://%s:%s\n' "$SERVER_HOST" "$SERVER_PORT"
printf '  F=键盘开关  │  Ctrl-C 退出\n\n'

ros2 run finav js_kb_router.py \
    --ros-args --params-file "$CONTROL_PARAMS" &
ROUTER_PID=$!
wait "$ROUTER_PID" || true
