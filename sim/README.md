# sim 使用说明

先执行一次环境准备：

```bash
colcon build --packages-select finav
source install/setup.bash
```

## PathPlan

启动：

```bash
ros2 launch finav path_plan_sim.launch.py
```

使用：

1. 在 RViz 用 2D Pose Estimate 设置起点。
2. 用 2D Goal Pose 设置终点。
3. 查看规划结果（/plan）。

## Map

启动：

```bash
ros2 launch finav map_sim.launch.py
```

同时另一个终端启动键盘控制：

```bash
ros2 run finav js_kb_router.py
```

建图完毕后保存地图，在一个新终端输入：

```bash
./scripts/tool/save_map.sh
```

使用：

1. 启动后在 RViz 进行建图。
2. 用遥控或速度指令驱动小车覆盖区域。
3. 建图完成后保存地图到 maps 目录。

## Nav

启动：

```bash
ros2 launch finav nav_sim.launch.py
```

指定地图启动（示例）：

```bash
ros2 launch finav nav_sim.launch.py
```

使用：

1. 在 RViz 先设置 2D Pose Estimate。
2. 再设置 2D Nav Goal。
3. 观察导航路径与底盘运动。
