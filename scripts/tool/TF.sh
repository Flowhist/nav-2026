#!/bin/bash
# 保存当前 TF 树到 figures/ 目录（PDF + GV 格式）
# 用法：bash scripts/tool/TF.sh
# 前提：ros2 launch finav map.launch.py（或任意 ROS2 节点）正在运行

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKSPACE_DIR="$(cd "$REPO_DIR/../.." && pwd)"
FIGURES_DIR="$REPO_DIR/figures"
mkdir -p "$FIGURES_DIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/local_setup.bash" 2>/dev/null || source "$WORKSPACE_DIR/install/setup.bash"

TIMESTAMP=$(date +"%Y-%m-%d_%H.%M.%S")
OUTPUT="$FIGURES_DIR/tf_tree_$TIMESTAMP"

echo "正在监听 TF 数据（5 秒）..."
cd "$FIGURES_DIR"
ros2 run tf2_tools view_frames

# view_frames 固定输出 frames_<timestamp>.pdf/.gv，重命名为含 tf_tree 前缀
LATEST_PDF=$(ls -t "$FIGURES_DIR"/frames_*.pdf 2>/dev/null | head -1)
LATEST_GV=$(ls  -t "$FIGURES_DIR"/frames_*.gv  2>/dev/null | head -1)

if [[ -z "$LATEST_PDF" ]]; then
    echo "❌ 未生成 TF 树文件，请确认 ROS2 节点正在运行"
    exit 1
fi

mv "$LATEST_PDF" "${OUTPUT}.pdf"
mv "$LATEST_GV"  "${OUTPUT}.gv"

echo "✓ TF 树已保存："
echo "  PDF : ${OUTPUT}.pdf"
echo "  GV  : ${OUTPUT}.gv"
