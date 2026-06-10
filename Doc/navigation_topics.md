# 导航话题信息梳理

本文基于当前 `finav` 代码中 `launch/nav.launch.py`、相关子 launch、`config/*.yaml` 和控制脚本整理。频率优先采用配置值；事件触发型话题标记为“事件触发”。实际频率可用 `ros2 topic hz <topic>` 在实机运行时复核。

## 1. 导航主链路

```text
RViz / nav_voice_bridge
  -> /goal_pose
  -> nav_path_plan.py
  -> /plan
  -> nav_control
  -> /nav_cmd_vel
  -> base_control_router
  -> /cmd_vel
  -> base_control
  -> CAN/电机
```

| 话题 | 类型 | 主要数据 | 发布者 | 订阅者 | 频率/触发时机 |
| --- | --- | --- | --- | --- | --- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 目标点位置 `pose.position.x/y` 和目标朝向 `pose.orientation`，坐标系为 `map` | RViz `2D Goal Pose` 或 `nav_voice_bridge.py` | `nav_path_plan.py` | 事件触发：设置目标点或语音/地点导航命中后发布一次 |
| `/plan` | `nav_msgs/msg/Path` | 全局路径点序列，`header.frame_id` 通常为 `map` | `nav_path_plan.py` | `nav_control.py`、RViz | 配置规划循环 `plan_rate_hz=2.0Hz`；新目标、地图更新或偏离路径触发重规划时发布；清空路径时可发布空 Path |
| `/nav_cmd_vel` | `geometry_msgs/msg/Twist` | 导航控制速度，主要使用 `linear.x` 和 `angular.z` | `nav_control.py` | `base_control_router.py` | 配置 `control_rate_hz=10.0Hz`；有有效路径时周期发布；到点、清空路径、重规划暂停时发布零速 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 最终底盘速度指令，主要使用 `linear.x` 和 `angular.z` | `base_control_router.py` | `base_control.py` | 配置 `router_rate=50.0Hz`；router 仲裁键盘、摇杆、Web、导航后周期发布 |
| `/nav_clear` | `std_msgs/msg/Empty` | 清空当前导航控制/规划状态 | `base_control_router.py` | `nav_path_plan.py`、`nav_control.py` | 事件触发：摇杆抢停导航、需要终止导航时发布；router 内部节流约 0.2s |
| `/base_control_router/status` | `std_msgs/msg/String` | 控制仲裁事件，例如 `joystick_stop:nav`、`joystick_reset`、`joystick_reset_required:nav ignored`、`base_fault_active`、`base_fault_cleared` | `base_control_router.py` | 语音/上层业务/调试终端 | 事件触发：抢停、复位、底盘故障变化或指令被拒绝时发布 |

## 2. 雷达与地图链路

| 话题 | 类型 | 主要数据 | 发布者 | 订阅者 | 频率/触发时机 |
| --- | --- | --- | --- | --- | --- |
| `/scan_left` | `sensor_msgs/msg/LaserScan` | 左雷达原始扫描，frame 为 `laser_left_frame` | `free_lidar_left_node` | `scan_fusion_node` | 雷达配置 `scan_frequency=30Hz`；实际受雷达输出和驱动处理影响 |
| `/scan_right` | `sensor_msgs/msg/LaserScan` | 右雷达原始扫描，frame 为 `laser_right_frame` | `free_lidar_right_node` | `scan_fusion_node` | 雷达配置 `scan_frequency=30Hz`；实际受雷达输出和驱动处理影响 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 融合后的车体坐标系扫描，frame 配置为 `base_link`，角度范围 `[-pi, pi]` | `scan_fusion_node` | `slam_toolbox`、RViz | 输入雷达同步融合后发布，理论接近雷达频率；受左右雷达同步、`input_timeout_sec=1.0`、TF 查询影响 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 栅格地图，包含 `info.resolution`、`origin`、`data` 占用概率 | `slam_toolbox` 定位模式 | `nav_path_plan.py`、RViz | 配置 `map_update_interval=1.0s`，约 `1Hz`；定位模式加载已有地图并发布给规划和 RViz |

## 3. 里程计、IMU 与定位链路

| 话题 | 类型 | 主要数据 | 发布者 | 订阅者 | 频率/触发时机 |
| --- | --- | --- | --- | --- | --- |
| `/odom_encoder` | `nav_msgs/msg/Odometry` | 编码器积分里程计；pose 为编码器积分，twist 含 `linear.x`、`angular.z` | `base_control.py` | `ekf_filter_node` | 配置 `update_rate=10.0Hz`；CAN 在线且轮速读取成功时发布 |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU 四元数姿态、角速度、线加速度；frame 为 `imu_link` | `dm_imu_publisher.py` | `ekf_filter_node` | 定时器 `0.01s`，上限 `100Hz`；仅在串口有新 IMU 数据时发布 |
| `/imu/rpy` | `geometry_msgs/msg/Vector3Stamped` | roll/pitch/yaw 调试量，可配置角度或弧度 | `dm_imu_publisher.py` | 调试/RViz 可选 | 同 `/imu/data`，上限 `100Hz`；`publish_rpy=true` 时发布 |
| `/odom` | `nav_msgs/msg/Odometry` | EKF 融合后的里程计；供定位使用 | `robot_localization/ekf_node`，由 `/odometry/filtered` remap 到 `/odom` | `slam_toolbox`、RViz | 配置 `frequency=10.0Hz` |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态 TF：`odom -> base_link`、`map -> odom` 等 | `ekf_filter_node`、`slam_toolbox`、`robot_state_publisher` | 全局 TF 消费者 | `ekf_filter_node` 约 `10Hz` 发布 `odom -> base_link`；`slam_toolbox` 按定位更新发布 `map -> odom`；机器人模型 TF 由 robot_state_publisher 发布 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态 TF：`base_link` 到雷达、IMU、车体模型固定坐标关系 | `robot_state_publisher` | 全局 TF 消费者 | 节点启动时发布一次，静态持久化 |

## 4. 底盘与控制仲裁链路

| 话题 | 类型 | 主要数据 | 发布者 | 订阅者 | 频率/触发时机 |
| --- | --- | --- | --- | --- | --- |
| `/js_state` | `std_msgs/msg/Bool` | 摇杆在线状态，`true` 表示 HID 摇杆连接/可用 | `joy_control.py` | `base_control_router.py` | 配置 `publish_rate=25.0Hz` |
| `/js_cmd_vel` | `geometry_msgs/msg/Twist` | 摇杆速度指令，主要使用 `linear.x` 和 `angular.z` | `joy_control.py` | `base_control_router.py` | 配置 `publish_rate=25.0Hz`；摇杆回中时发布零速 |
| `/web_cmd_vel` | `geometry_msgs/msg/Twist` | Web 控制速度指令 | Web/ROS bridge 侧 | `base_control_router.py` | Web 控制事件触发或按其发送周期发布；router 侧超时参数 `web_cmd_timeout=0.4s` |
| `/base_status` | `std_msgs/msg/String` | CAN 连接状态 JSON，字段包括 `connected`、`reconnecting`、`can_errors`、`can_lost_at`、`can_recovered_at` | `CanConnectionManager`，嵌入 `base_control.py` | 调试/上层状态显示 | 看门狗 `1Hz` 周期发布；连接、断开、shutdown 时也会发布 |

## 5. 地点导航与 RViz 可视化

| 话题 | 类型 | 主要数据 | 发布者 | 订阅者 | 频率/触发时机 |
| --- | --- | --- | --- | --- | --- |
| `/nav_voice_bridge/voice_command` | `std_msgs/msg/String` | 地点名称文本，例如 `.locations.yaml` 中的 key | 语音/上层业务 | `nav_voice_bridge.py` | 事件触发；仅 `nav.launch.py use_nav_bridge:=true` 时启用 |
| `/nav_voice_bridge/status` | `std_msgs/msg/String` | 地点指令处理结果，例如 unknown location、navigating to xxx；不代表完整导航成功/失败 | `nav_voice_bridge.py` | 上层业务/调试终端 | 事件触发：收到地点指令并处理后发布 |
| `/locations` | `visualization_msgs/msg/MarkerArray` | 地图地点标记、箭头和文字标签 | `annotate_visualizer.py` | RViz | 节点启动加载 `.locations.yaml` 后发布一次；marker lifetime 为 0，RViz 中持续显示 |

## 6. 关键节点关系

| 节点 | 主要输入 | 主要输出 | 说明 |
| --- | --- | --- | --- |
| `free_lidar_left_node` / `free_lidar_right_node` | 两颗 FREE 雷达 TCP 数据 | `/scan_left`、`/scan_right` | 雷达 IP 来自 `config/lidar.yaml`，网络端口为 TCP `2111` |
| `scan_fusion_node` | `/scan_left`、`/scan_right`、TF | `/scan` | 将双雷达点云转换/融合到 `base_link` |
| `slam_toolbox` | `/scan`、`/odom`、地图文件 | `/map`、`/tf` 中的 `map -> odom` | `nav.launch.py` 中以 localization 模式启动 |
| `nav_path_plan.py` | `/map`、`/goal_pose`、TF `map -> base_link`、`/nav_clear` | `/plan` | 基于 OccupancyGrid 和车体足迹做全局规划 |
| `nav_control.py` | `/plan`、TF `map -> base_link`、`/nav_clear` | `/nav_cmd_vel` | Pure Pursuit + ROTATE/DRIVE 状态控制 |
| `base_control_router.py` | `/nav_cmd_vel`、`/js_cmd_vel`、`/web_cmd_vel`、键盘输入 | `/cmd_vel`、`/nav_clear` | 唯一最终速度仲裁器；摇杆非零可抢停键盘/Web/导航 |
| `base_control.py` | `/cmd_vel`、CAN 轮速反馈 | `/odom_encoder`、`/base_status` | 独占 CAN，向电机下发速度并发布编码器里程计 |
| `ekf_filter_node` | `/odom_encoder`、`/imu/data` | `/odom`、`/tf` 中的 `odom -> base_link` | 当前配置使用编码器 vx/vyaw 和 IMU yaw |

## 7. 运行时复核命令

```bash
# 查看当前所有话题和类型
ros2 topic list -t

# 查看关键话题频率
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic hz /odom
ros2 topic hz /odom_encoder
ros2 topic hz /nav_cmd_vel
ros2 topic hz /cmd_vel
ros2 topic hz /base_status

# 查看消息内容
ros2 topic echo /goal_pose
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel
ros2 topic echo /base_status

# 查看 TF 关系
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```
