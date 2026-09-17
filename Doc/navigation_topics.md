# 导航接口与路线行为

## 控制链路

```text
/goal_pose 或 /nav_task/command
  -> nav_task_manager + route_tasks
  -> /nav_task/planner_goal -> nav_path_plan
  -> /nav_task/planner_path -> nav_task_manager
  -> /plan -> nav_control -> /nav_cmd_vel
  -> base_control_router -> /cmd_vel -> 电机
```

节点由 `launch/nav.launch.py` 启动。任务管理器处理目标、路线及状态，规划器生成路径，原有 Pure Pursuit 控制器跟踪路径，底盘仲裁器决定最终运动指令。

## 常用接口

类型省略 `/msg/`，速度单位为 m/s、rad/s。

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 单点导航目标，坐标系 `map` |
| `/nav_task/command` | `std_msgs/String` | 路线命令 JSON，见下文 |
| `/nav_status` | `std_msgs/String` | 任务状态 JSON |
| `/plan` | `nav_msgs/Path` | 交给控制器的完整执行路径 |
| `/planning_map` | `nav_msgs/OccupancyGrid` | 含禁行区与膨胀约束的规划栅格 |
| `/nav_cmd_vel`、`/web_cmd_vel`、`/js_cmd_vel` | `geometry_msgs/Twist` | 导航、网页、手柄的仲裁输入 |
| `/cmd_vel` | `geometry_msgs/Twist` | 仲裁器输出，外部功能不要绕过仲裁直接发布 |
| `/nav_clear`、`/nav_clear_reason` | `std_msgs/Empty`、`std_msgs/String` | 取消导航及原因 |
| `/base_fault`、`/js_state` | `std_msgs/Bool` | 底盘故障、手柄链路状态 |
| `/handle/gear` | `std_msgs/UInt16` | 手柄档位 1～5 |
| `/nav_task/speed_limit` | `std_msgs/Float32` | 路线速度限制 |

规划器的 goal/path/status 与跟踪器 status 使用 `/nav_task/` 内部话题，不作为外部目标入口。可选地点语音桥接通过 `/nav_voice_bridge/voice_command` 接收地点名称，查 `.locations.yaml` 后发布 `/goal_pose`。

## 地图、定位与 TF

| 话题 / 变换 | 来源与用途 |
| --- | --- |
| `/scan_left`、`/scan_right` | 两台 HE-3051 原始 LaserScan |
| `/scan` | 融合到 `base_link` 的 LaserScan；SLAM、Web、RViz 共用 |
| `/odom_encoder` | 底盘编码器 Odometry；供 EKF 和手柄速度显示使用 |
| `/imu/data` | DM-IMU 的 Imu 数据 |
| `/odom` | EKF 融合的 Odometry |
| `/map` | SLAM Toolbox 的 OccupancyGrid |
| `map -> odom` | SLAM Toolbox 建图/定位发布 |
| `odom -> base_link` | EKF 发布 |
| `base_link -> laser_*_frame / imu_link` | URDF 固定变换 |

## 开放路线

网页先选择地图并启动导航，再选择已保存路线。HTTP 入口为 `POST /api/nav/route`，ROS 命令使用相同 JSON：

```json
{"action":"start","map_name":"地图名","route_id":"路线ID"}
```

`action` 支持 `start`、`pause`、`resume`、`cancel`。任务状态包括 `PLANNING`、`FOLLOWING`、`REACHED`、`FAILED`、`CANCELED`、`PAUSED`。

- 从当前位置规划到附近可达入线点，拼接剩余路线，交给原有 `/plan` 控制器。
- 不要求先到路线起点；反向只反转点列，车头仍朝行进方向；不逐点停靠，到端点停车。
- 摇杆接管取消任务。人工继续时重新规划接入路径；保存路线后自动重新生成路径，已暂停任务保持暂停。
- 导航速度受手柄档位比例 `gear / 5`、路线限速及底盘上限约束。
- 支持地图编辑禁行区；不支持闭环计圈，本功能没有新增动态障碍检测或自动绕行。

## 只读调试

先 source ROS 与工作区环境，在相应服务运行时执行：

```bash
ros2 topic list -t
ros2 topic echo /nav_status --once
ros2 topic echo /plan --once
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic hz /odom_encoder
ros2 run tf2_ros tf2_echo map base_link
```
