# Path Planner 仿真测试套件

用于在单地图（默认 `map4`）上离线测试 `path_plan.py` 路径生成效果，不依赖真车。

## 启动

1. 构建并加载环境

```bash
colcon build --packages-select finav
source install/setup.bash
```

2. 启动仿真

```bash
ros2 launch finav path_plan_sim.launch.py
```

可选参数：

```bash
ros2 launch finav path_plan_sim.launch.py \
  map_yaml:=/home/flowhist/workspace/final_ws/src/finav/maps/map4/map4.yaml \
  use_rviz:=true \
  use_cmd_vel:=false
```

## 在 RViz 中测试

1. 使用 `2D Pose Estimate` 在地图上设置虚拟车起点（发布到 `/initialpose`）。
2. 使用 `2D Goal Pose` 设置目标点（发布到 `/goal_pose`）。
3. 查看 `/plan`（红线）路径效果。

## 命令行发目标点（可选）

```bash
ros2 run finav sim_send_goal.py --x 2.0 --y -1.5 --yaw-deg 0
```

## 文件说明

- `launch/path_plan_sim.launch.py`：仿真入口（用于 `ros2 launch finav path_plan_sim.launch.py`）
- `sim/launch/path_plan_sim.launch.py`：仿真配置原始版本
- `sim/scripts/sim_fake_pose_tf.py`：虚拟位姿 TF（`map->base_link`）
- `sim/scripts/sim_send_goal.py`：命令行目标点发送工具
- `sim/config/path_plan_sim.yaml`：规划参数
- `sim/config/sim_fake_pose.yaml`：虚拟位姿参数
- `sim/rviz/path_plan_sim.rviz`：可视化配置
