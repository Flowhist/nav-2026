# Path Planner 仿真测试套件

用于在单地图（默认 `map4`）上离线测试 `path_plan.py` 路径生成效果，不依赖真车。

另外已补充（严格对齐实机主链路）：

- `gazebo_mapping_sim.launch.py`：Gazebo 从零建图仿真（无先验地图）
- `gazebo_navigation_sim.launch.py`：Gazebo 导航仿真（基于已有地图定位）

## 启动

1. 构建并加载环境

```bash
colcon build --packages-select finav
source install/setup.bash
```

2. 启动仿真

```bash
ros2 launch finav sim/launch/path_plan_sim.launch.py
```

建图仿真：

```bash
ros2 launch finav sim/launch/gazebo_mapping_sim.launch.py
```

导航仿真：

```bash
ros2 launch finav sim/launch/gazebo_navigation_sim.launch.py
```

Gazebo 建图仿真（无先验地图）：

```bash
ros2 launch finav sim/launch/gazebo_mapping_sim.launch.py
```

说明：

- 推荐从零建图：`sim/launch/gazebo_mapping_sim.launch.py`。
- 该链路使用 Gazebo 真实传感器插件输出 `/scan`、`/imu/data`、`/odom`，由 SLAM 直接建图。
- 导航仿真使用 Gazebo 传感器 + SLAM localization（需 `map_file` 对应已保存地图）。

## 可选参数

路径规划仿真：

```bash
ros2 launch finav sim/launch/path_plan_sim.launch.py \
  map_yaml:=/home/flowhist/workspace/final_ws/src/finav/maps/map4/map4.yaml \
  use_rviz:=true \
  use_cmd_vel:=false
```

Gazebo 建图仿真：

```bash
ros2 launch finav sim/launch/gazebo_mapping_sim.launch.py \
  world:=/home/flowhist/workspace/finav_ws/src/finav/sim/gazebo/worlds/office_corridor_sketch.world \
  use_rviz:=true \
  x:=-0.15 y:=-5.65 yaw:=1.5707963
```

Gazebo 导航仿真：

```bash
ros2 launch finav sim/launch/gazebo_navigation_sim.launch.py \
  world:=/home/flowhist/workspace/finav_ws/src/finav/sim/gazebo/worlds/office_corridor_sketch.world \
  map_file:=map4 \
  maps_dir:=/home/flowhist/workspace/finav_ws/src/finav/maps \
  use_rviz:=true \
  x:=-0.15 y:=-5.65 yaw:=1.5707963
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

- `sim/launch/path_plan_sim.launch.py`：路径规划仿真入口
- `sim/launch/gazebo_mapping_sim.launch.py`：Gazebo 建图仿真入口
- `sim/launch/gazebo_navigation_sim.launch.py`：Gazebo 导航仿真入口
- `sim/scripts/sim_chassis_sensor_pub.py`：虚拟底盘与传感器源（可输出 `/odom_encoder`、`/imu/data`、`/joint_states`）
- `sim/scripts/sim_send_goal.py`：命令行目标点发送工具
- `sim/config/path_plan_sim.yaml`：规划参数
- `sim/config/sim_fake_pose.yaml`：虚拟底盘状态参数
- `sim/rviz/path_plan_sim.rviz`：可视化配置
