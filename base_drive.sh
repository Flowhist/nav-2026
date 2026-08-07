#!/usr/bin/env bash
# base_control.sh
# 同时启动 base_control + STM32 手柄 + base_control_router
# 键盘控制已集成：F 键开关，W/S 前后，A/D 旋转，J/K 换档，空格急停
# 使用方式：bash base_control.sh
#         bash base_control.sh --handle-port /dev/ttyUSB0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR"
WORKSPACE_DIR="$(cd "$REPO_DIR/../.." && pwd)"

# ── 参数解析 ──────────────────────────────────────────────────────────── #
HANDLE_PORT="/dev/ttyUSB0"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --handle-port) HANDLE_PORT="$2"; shift 2 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# ── 清理函数（Ctrl-C / 退出时调用）──────────────────────────────────── #
DRIVER_PID=""
HANDLE_PID=""
STATUS_TAIL_PID=""
cleanup() {
    printf '\033[?25h'
    printf '\n\n\n正在停止节点...\n'
    [[ -n "$STATUS_TAIL_PID" ]] && kill "$STATUS_TAIL_PID" 2>/dev/null || true
    [[ -n "$DRIVER_PID" ]] && kill "$DRIVER_PID" 2>/dev/null || true
    [[ -n "$HANDLE_PID" ]] && kill "$HANDLE_PID" 2>/dev/null || true
    wait "$STATUS_TAIL_PID" "$DRIVER_PID" "$HANDLE_PID" 2>/dev/null || true
    printf '已停止。' 
}
trap cleanup EXIT INT TERM

# ── Source ROS 环境 ───────────────────────────────────────────────────── #
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP_LOCAL="$WORKSPACE_DIR/install/local_setup.bash"
WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"

[[ -f "$ROS_SETUP" ]] || {
    printf '\033[31m未找到 %s，请先安装 ROS 2 Humble。\033[0m\n' "$ROS_SETUP" >&2
    exit 1
}

set +u
source "$ROS_SETUP"
if [[ -f "$WORKSPACE_SETUP_LOCAL" ]]; then
    source "$WORKSPACE_SETUP_LOCAL"
elif [[ -f "$WORKSPACE_SETUP" ]]; then
    source "$WORKSPACE_SETUP"
else
    set -u
    printf '\033[31m未找到工作区 install/setup.bash，请先在 %s 执行 colcon build。\033[0m\n' "$WORKSPACE_DIR" >&2
    exit 1
fi
set -u

command -v ros2 >/dev/null 2>&1 || {
    printf '\033[31m当前环境没有 ros2 命令，ROS 环境未正确加载。\033[0m\n' >&2
    exit 1
}

# ── 抑制 Fast-DDS XML 解析警告 ─────────────────────────────────────── #
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO_DIR/config/fastdds_profiles.xml"

# ── 清理残留旧进程（防止 CAN 占用冲突）──────────────────────────────── #
printf '清理旧进程...\n'
pkill -9 -f "base_control.py" 2>/dev/null || true
pkill -9 -f "handle_control.py" 2>/dev/null || true
pkill -9 -f "base_control_router.py" 2>/dev/null || true
sleep 1

# ── 启动底盘驱动节点 ─────────────────────────────────────────────────── #
WHILL_PARAMS="$REPO_DIR/config/base_control.yaml"
[[ -f "$WHILL_PARAMS" ]] || {
    printf '\033[31m未找到参数文件: %s\033[0m\n' "$WHILL_PARAMS" >&2
    exit 1
}

printf '▶ 启动 base_control\n'
: > /tmp/base_control.log
ros2 run finav base_control.py \
    --ros-args --params-file "$WHILL_PARAMS" \
    > /tmp/base_control.log 2>&1 &
DRIVER_PID=$!
tail -n 0 -F /tmp/base_control.log 2>/dev/null \
    | grep --line-buffered -E "急停|STO|33555|控制指令已复位|底盘故障|非零 /cmd_vel|下发轮速失败" &
STATUS_TAIL_PID=$!

printf '▶ 启动 STM32 手柄\n'
ros2 launch finav handle.launch.py "handle_port:=$HANDLE_PORT" \
    > /dev/null 2>&1 &
HANDLE_PID=$!

printf '等待节点就绪...\n'
sleep 2.5

# ── 进程存活检查 ─────────────────────────────────────────────────────── #
if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
    printf '\033[31m✗ base_control 启动失败'
    exit 1
fi
if ! kill -0 "$HANDLE_PID" 2>/dev/null; then
    printf '\033[31m✗ handle.launch.py 启动失败'
    exit 1
fi

printf '\033[32m✓ 两个节点均已启动\033[0m\n'
printf '  手柄串口: %s  │  F=键盘开关  │  Ctrl-C 退出\n\n' "$HANDLE_PORT"
printf '\033[?25l'   # 隐藏光标

# ── 实时状态监视器 + 键盘控制 ────────────────────── #
ros2 run finav base_control_router.py --ros-args --params-file "$WHILL_PARAMS" 2>&1 | grep -v "XMLPARSER Error" || true
