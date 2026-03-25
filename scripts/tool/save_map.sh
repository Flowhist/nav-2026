#!/usr/bin/env bash
# save_map.sh
# 保存当前 SLAM 地图（调用 scripts/tool/manual_save_map.py）
#
# 用法：
#   bash scripts/tool/save_map.sh                     # 默认地图名 manual_map
#   bash scripts/tool/save_map.sh <地图名>            # 指定地图名
#   bash scripts/tool/save_map.sh <地图名> --cleanup  # 保存后停止所有建图进程
#
# 前提：map.launch.py 已在运行（slam_toolbox 服务可用）

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKSPACE_DIR="$(cd "$REPO_DIR/../.." && pwd)"

# ── 参数解析 ──────────────────────────────────────────────────────────── #
MAP_NAME=""
DO_CLEANUP=false

for arg in "$@"; do
    case "$arg" in
        --cleanup) DO_CLEANUP=true ;;
        --*) printf '未知参数: %s\n' "$arg"; exit 1 ;;
        *)   MAP_NAME="$arg" ;;
    esac
done

# 交互终端时若未指定地图名则提示输入
if [[ -z "$MAP_NAME" && -t 0 ]]; then
    read -rp "请输入地图名（回车使用默认 'manual_map'）: " MAP_NAME
fi
MAP_NAME="${MAP_NAME:-manual_map}"

# ── Source ROS 环境 ───────────────────────────────────────────────────── #
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WORKSPACE_DIR/install/local_setup.bash" 2>/dev/null || source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true

# ── 抑制 Fast-DDS XML 解析警告 ─────────────────────────────────────── #
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO_DIR/config/fastdds_profiles.xml"

# ── 保存地图 ─────────────────────────────────────────────────────────── #
SAVE_SCRIPT="$REPO_DIR/scripts/tool/manual_save_map.py"

if [[ ! -f "$SAVE_SCRIPT" ]]; then
    printf '[ERROR] 未找到保存脚本: %s\n' "$SAVE_SCRIPT"
    exit 1
fi

printf '▶ 保存地图: %s\n' "$MAP_NAME"
python3 "$SAVE_SCRIPT" --name "$MAP_NAME"

# ── 清理建图进程（可选）─────────────────────────────────────────────── #
if [[ "$DO_CLEANUP" == true ]]; then
    printf '\n▶ 停止建图进程（map.launch.py）...\n'
    bash "$SCRIPT_DIR/clean_map.sh"
fi
