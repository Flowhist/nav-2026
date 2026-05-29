# Finav 项目 Wiki

本文面向新加入开发人员，用于快速理解 Finav 的系统组成、运行链路、关键文件位置和常见开发入口。项目是一个基于 ROS 2 Humble 的实机导航系统，包含底盘控制、FREE 激光雷达、DM-IMU、EKF 融合、SLAM Toolbox 建图/定位、自研路径规划/路径跟踪、Web 调试后台和仿真环境。

## 1. 系统总览

Finav 的实机主链路可以概括为：

```text
WHILL 底盘反馈 -> /odom_encoder \
                               -> robot_localization EKF -> /odom + odom->base_link
DM-IMU --------> /imu/data     /

FREE 左/右雷达 -> /scan_left, /scan_right -> scan_fusion_node -> /scan
URDF ----------> base_link -> laser_left_frame / laser_right_frame / imu_link 等静态 TF

/scan + /odom + TF -> slam_toolbox
  建图模式: 输出 /map 和 map->odom
  定位模式: 加载 maps/<map>/<map>，输出 map->odom

/map + /goal_pose + TF -> path_plan.py -> /plan
/plan + TF -----------> nav_control.py -> /nav_cmd_vel -> base_control_router.py -> /cmd_vel
/cmd_vel -------------> base_control.py -> WHILL 电机

server/ros_bridge.py 订阅 ROS 状态并提供 Web 页面调试、建图、导航和配置编辑入口。
```

核心坐标系：

- `map`：全局地图坐标系，由 SLAM Toolbox 发布到 `odom` 的变换。
- `odom`：连续里程计坐标系，由 EKF 发布到 `base_link` 的变换。
- `base_link`：车体基座坐标系。
- `laser_left_frame` / `laser_right_frame`：实机双雷达坐标系，来自 URDF 静态 TF。
- `laser_frame`：保留给旧配置或仿真兼容使用。
- `imu_link`：IMU 坐标系，来自 URDF 静态 TF。

核心话题：

- `/scan_left` / `/scan_right`：左右 FREE 雷达原始 `sensor_msgs/LaserScan`。
- `/scan`：双雷达融合后的导航/建图输入，仍是下游统一接口。
- `/imu/data`：DM-IMU `sensor_msgs/Imu`。
- `/odom_encoder`：底盘编码器里程计。
- `/odom`：EKF 融合里程计。
- `/map`：SLAM 输出占据栅格。
- `/goal_pose`：导航目标。
- `/plan`：自研规划器输出路径。
- `/cmd_vel`：底盘速度指令。

## 2. 主要运行入口

### 2.1 一键实机调试入口

文件：`start_finav.sh`

作用：

- 加载 ROS 2 和工作区环境。
- 设置 `FASTRTPS_DEFAULT_PROFILES_FILE`、`FINAV_REPO_DIR`、`FINAV_MAPS_DIR`。
- 启动底盘驱动 `base_control.py`。
- 启动 HID 摇杆链路 `launch/sub/joy.launch.py`。
- 启动 Web 后台 `server/run_server.py`。
- 启动键盘/摇杆路由 `base_control_router.py`。
- 退出时清理建图/导航相关进程。

典型用法：

```bash
bash start_finav.sh
bash start_finav.sh --joy-dev /dev/input/js0 --host 0.0.0.0 --port 8010
```

### 2.2 仅启动底盘与遥控

文件：`base_drive.sh`

作用：

- 启动 `base_control.py` 和 HID 摇杆链路。
- 前台运行 `base_control_router.py`，用于键盘接管、档位切换和急停。
- 适合底盘联调，不启动 Web 和 SLAM。

### 2.3 建图入口

文件：`launch/map.launch.py`

组成：

- `launch/sub/lidar.launch.py`：FREE 雷达。
- `launch/sub/dm_imu.launch.py`：DM-IMU。
- `launch/sub/ekf.launch.py`：EKF 融合。
- `launch/sub/slam_toolbox.launch.py mode:=mapping`：SLAM Toolbox 建图。
- `launch/sub/robot_model.launch.py`：URDF 和静态 TF。

典型用法：

```bash
ros2 launch finav map.launch.py
```

建图阶段默认不自动启动 RViz，项目更偏向通过 Web 页面查看实时地图。

### 2.4 导航入口

文件：`launch/nav.launch.py`

组成：

- 雷达、IMU、EKF、机器人模型。
- `slam_toolbox` 定位模式：加载已有地图。
- `path_plan.py`：自研全局规划。
- `nav_control.py`：路径跟踪并发布 `/cmd_vel`。

典型用法：

```bash
ros2 launch finav nav.launch.py
ros2 launch finav nav.launch.py map_file:=map4
```

地图目录默认来自 `FINAV_MAPS_DIR`，否则使用仓库下 `maps/`。地图结构约定为：

```text
maps/<map_name>/<map_name>.yaml
maps/<map_name>/<map_name>.pgm
```

### 2.5 Web 后台入口

文件：`server/run_server.py`

典型用法：

```bash
python3 server/run_server.py --host 0.0.0.0 --port 8010
```

更常用的方式是通过 `start_finav.sh` 启动。

## 3. 模块组成与大致原理

### 3.1 底盘控制模块

相关文件：

- `scripts/control/base_control.py`
- `scripts/control/joy_control.py`
- `scripts/control/base_control_router.py`
- `config/base_control.yaml`
- `config/joy.yaml`
- `base_drive.sh`
- `start_finav.sh`

原理：

- `base_control.py` 通过 omnilibs CAN 驱动连接 WHILL 底盘，CAN 连接管理已整合为节点内的内联逻辑。
- 订阅 `/cmd_vel`，将线速度和角速度换算为左右轮角速度，下发到底盘。
- 读取左右轮反馈，积分发布 `/odom_encoder`。
- 不直接发布 `odom -> base_link` TF，TF 由 EKF 输出，避免多个节点重复发布同一变换。
- `joy_control.py` 订阅 `joy_node` 发布的 `/joy`，按 `config/joy.yaml` 的轴向、死区、饱和区和两档分界线，以及 `config/base_control.yaml` 中的摇杆速度档位，转换为 `/js_cmd_vel`，并发布 `/js_state`。
- `base_control_router.py` 负责摇杆/键盘/Web 控制仲裁，最终发布 `/cmd_vel`。

#### 故障检测与急停处理

急停旋钮按下后电机进入 STO/fault，`base_control.py` 通过两层机制快速响应：

**主动监测（5Hz `_monitor_fault`）**
- 每 200ms 调用 `driver.get_fault_status()`，利用 TPDO 缓存读取电机错误寄存器（非阻塞，无 CAN 总线等待）。
- 检测到故障码非零后立即调用 `_latch_motion_fault()`，清空内部控制指令缓存并设置 `_require_cmd_reset=True`。
- 同时发布 `Bool` 消息到 `/base_fault`，供下游节点感知故障状态。
- 响应延迟 < 200ms，远快于等待 `move_velocity` SDO 超时（2-3s）。

**被动防护（`_send_wheel_velocity` 异常捕获）**
- 如果主动监测漏过（如 TPDO 未及时更新），`move_velocity` 内部抛出异常时同样会触发 `_latch_motion_fault`。

**故障锁存与复位**
- 故障后，`_require_cmd_reset=True` 会拦截所有非零 `/cmd_vel`，日志提示请先发送 0 速复位。
- 操作者释放急停后，由上游（router/web/joystick）先发送一次零速 `/cmd_vel` 来清除锁存。
- 清除锁存时，`_monitor_fault` 进入 2 秒冷却期，避免电机未完全恢复时被重复锁存。

**故障后尝试刹停**
- `_try_stop_after_fault` 以 1 秒间隔尝试发送零速到电机（限频避免反复阻塞在 CANopen 状态机超时上）。

**话题**：
- `/base_fault`（`std_msgs/Bool`）：True = 底盘检测到电机故障，False = 故障已清除。

开发入口：

- 改底盘速度上限、轮距、CAN 通道：`config/base_control.yaml`。
- 改底盘驱动和里程计逻辑：`scripts/control/base_control.py`。
- 改故障检测频率或冷却期：`scripts/control/base_control.py` 中 `_monitor_fault` 定时器周期和 `_fault_monitor_cooldown_until`。
- 改 HID 摇杆轴向、死区和档位分界线：`config/joy.yaml`。
- 改 HID 摇杆速度档位：`config/base_control.yaml`。
- 改键盘/Web 仲裁：`scripts/control/base_control_router.py`。

### 3.2 FREE 雷达模块

相关文件：

- `third_party/free_lidar/src/rosnode/free_lidar_node.cpp`
- `third_party/free_lidar/src/driver/free_eth_driver.cpp`
- `third_party/free_lidar/src/driver/trailing_filter.cpp`
- `third_party/free_lidar/include/free_lidar/*.h`
- `src/rosnode/scan_fusion_node.cpp`
- `launch/sub/lidar.launch.py`
- `config/lidar.yaml`
- `third_party/free_lidar/README.upstream.md`
- `third_party/free_lidar/docs/H1E0-06B_第四章设备连接.md`

原理：

- 当前实机链路使用两台以太网 FREE 雷达，CMake 中定义了 `FREE_LIDAR_ENABLE_SERIAL=0`。
- `free_eth_driver.cpp` 负责 TCP 连接雷达、登录设备、配置扫描角度/频率/分辨率、解析扫描分包。
- `free_lidar_node.cpp` 分别发布 `/scan_left` 和 `/scan_right`。
- `scan_fusion_node.cpp` 将左右 LaserScan 变换到 `base_link`，按角度栅格取最近距离，发布融合后的 `/scan`。
- 下游 SLAM、Web、RViz 和导航仍只依赖 `/scan`，不要随意改动外部话题契约。
- 当前雷达已正装朝上，`is_reverse_postion` 默认应保持 `false`。
- 雷达 QoS 使用 Best Effort，RViz 的 LaserScan Display 也必须设置为 Best Effort，否则会出现 Reliability QoS 不兼容。

关键配置：

- `left.scanner_ip` / `right.scanner_ip`：左右雷达 IP，当前常用 `192.168.1.111` 和 `192.168.1.112`。
- `left.frame_id` / `right.frame_id`：通常为 `laser_left_frame`、`laser_right_frame`。
- `left.topic_name` / `right.topic_name`：原始话题，默认 `/scan_left`、`/scan_right`。
- `scan_frequency`：扫描频率。
- `start_angle` / `stop_angle` / `offset_angle`：每个雷达的发布角度范围和偏置，可用于减少重合、扩大覆盖。
- `is_reverse_postion`：物理反装或点序相反时反序 ranges；当前正装时保持 `false`。
- `fusion.output_topic` / `fusion.output_frame`：融合输出话题和坐标系，默认 `/scan`、`base_link`。
- `fusion.angle_*`、`fusion.range_*`、`sync_queue_size`、`tf_timeout_sec`：融合角度分辨率、距离范围、同步队列和 TF 等待时间。
- `filter_switch`、`single_filter_enable`：厂商滤波控制。排查原始点云时应关闭。
- `use_recv_time_stamp`：是否使用首包接收时间作为 `LaserScan.header.stamp`，用于排查运动畸变。

常见排查：

- 连接失败先查网络：上位机网卡应与两台雷达在同一网段，分别 `ping 192.168.1.111`、`ping 192.168.1.112`。
- 分别执行 `ros2 topic hz /scan_left --qos-reliability best_effort`、`/scan_right`、`/scan` 确认原始和融合数据。
- `ros2 topic info /scan -v` 确认 `/scan` 发布者是 `scan_fusion_node`，QoS 为 Best Effort。
- `ros2 run tf2_ros tf2_echo base_link laser_left_frame` 和 `laser_right_frame` 确认 TF 存在。
- RViz 看不到点时确认 LaserScan 的 Reliability Policy 是 Best Effort。
- 排查完整链路时 RViz Fixed Frame 优先设为 `base_link` 或 `map`；只看单个原始雷达时用对应 `laser_left_frame` / `laser_right_frame`。

### 3.3 DM-IMU 模块

相关文件：

- `scripts/imu/dm_imu_publisher.py`
- `scripts/imu/config_dm_imu.py`
- `launch/sub/dm_imu.launch.py`
- `config/imu.yaml`
- `third_party/dm_imu/`

原理：

- 通过 USB 串口读取 DM-IMU 数据。
- 发布 `/imu/data` 和可选 `/imu/rpy`。
- 支持陀螺仪/加速度零偏、yaw 偏置和 yaw 漂移补偿。
- IMU 的 `frame_id` 通常为 `imu_link`，与 URDF 中静态 TF 对齐。

开发入口：

- 串口、波特率、零偏和补偿参数：`config/imu.yaml`。
- IMU 数据解析和发布逻辑：`scripts/imu/dm_imu_publisher.py`。
- 厂商串口模块：`third_party/dm_imu/dm_imu_modules/`。

### 3.4 EKF 融合模块

相关文件：

- `launch/sub/ekf.launch.py`
- `config/ekf.yaml`

原理：

- 使用 `robot_localization/ekf_node`。
- 输入 `/odom_encoder` 的前向速度 `vx` 和角速度 `vyaw`。
- 输入 `/imu/data` 的 z 轴角速度。
- 输出 `/odom`，并发布 `odom -> base_link` TF。
- 当前配置不融合编码器位置，也不融合 IMU 绝对 yaw，避免固定初始航向偏置。

开发入口：

- 调整融合源、协方差、频率：`config/ekf.yaml`。
- 如果出现里程尺度偏差，优先单独验证 `/odom_encoder`，再看 EKF 输出 `/odom`。

### 3.5 机器人模型与 TF

相关文件：

- `urdf/whillcar.urdf`
- `launch/sub/robot_model.launch.py`
- `rviz/mapping.rviz`
- `rviz/navigation.rviz`

原理：

- `robot_state_publisher` 从 URDF 发布静态/动态 TF。
- 当前关键固定关系包括 `base_link -> laser_left_frame`、`base_link -> laser_right_frame`、`base_link -> imu_link`。`laser_frame` 仅保留给旧配置或仿真兼容。
- RViz 中任何 Fixed Frame 不等于消息自身 frame 时，都需要 TF 树中存在对应变换。

开发入口：

- 改雷达/IMU 安装位姿：`urdf/whillcar.urdf`。
- 改 RViz 显示：`rviz/mapping.rviz`、`rviz/navigation.rviz`。

### 3.6 SLAM Toolbox 建图与定位

相关文件：

- `launch/sub/slam_toolbox.launch.py`
- `config/slam_toolbox_map.yaml`
- `config/slam_toolbox_nav.yaml`
- `scripts/tool/save_map.sh`
- `scripts/tool/manual_save_map.py`
- `scripts/tool/clean_map.sh`
- `scripts/tool/clean_nav.sh`
- `maps/`

原理：

- 建图模式使用 `sync_slam_toolbox_node`，输入 `/scan`、`/odom` 和 TF，输出 `/map` 与 `map -> odom`。
- 定位模式使用 `localization_slam_toolbox_node`，加载 `maps/<map>/<map>`，根据激光和里程计定位。
- `slam_toolbox.launch.py` 用 `TimerAction` 延迟启动 SLAM，等待传感器和 TF 稳定。

开发入口：

- 建图参数：`config/slam_toolbox_map.yaml`。
- 定位参数：`config/slam_toolbox_nav.yaml`。
- 地图保存和清理脚本：`scripts/tool/`。

### 3.7 自研路径规划模块

相关文件：

- `scripts/control/path_plan.py`
- `config/path_plan.yaml`

原理：

- 订阅 `/map` 和 `/goal_pose`。
- 通过 TF 查询当前 `map -> base_link`。
- 将 OccupancyGrid 转成可碰撞检测的栅格，并按真实矩形车体足迹采样。
- 使用离散航向的 SE2 A* 搜索路径，比单纯二维栅格更能约束车体朝向和转弯。
- 规划结果发布为 `/plan`，类型为 `nav_msgs/Path`。
- 支持目标变化、偏离路径触发重规划、路径 shortcut 平滑和路径点重采样。

开发入口：

- 车体尺寸、膨胀、安全边距、搜索上限：`config/path_plan.yaml`。
- A*、足迹碰撞、路径平滑：`scripts/control/path_plan.py`。

### 3.8 路径跟踪与导航控制模块

相关文件：

- `scripts/control/nav_control.py`
- `config/nav.yaml`

原理：

- 订阅 `/plan`，通过 TF 获取机器人在 `map` 下的位姿。
- 使用 Pure Pursuit 计算前瞻目标点。
- 根据航向误差在 DRIVE 和 ROTATE 两种状态间切换。
- DRIVE 模式输出线速度和角速度，ROTATE 模式原地旋转对准。
- 路径结束或收到 `/nav_clear` 后停车并重置。

开发入口：

- 线速度、角速度、Pure Pursuit 预瞄距离、到点阈值：`config/nav.yaml`。
- 控制状态机、前瞻点和角速度计算：`scripts/control/nav_control.py`。

### 3.9 Web 调试后台

相关文件：

- `server/run_server.py`
- `server/server_app.py`
- `server/ros_bridge.py`
- `server/process_manager.py`
- `server/state_store.py`
- `server/map_utils.py`
- `server/web/index.html`
- `server/web/styles.css`
- `server/web/app-core.js`
- `server/web/app-dock.js`
- `server/web/app-pages.js`
- `server/web/app.js`

原理：

- `run_server.py` 启动单进程 HTTP 服务。
- `server_app.py` 处理 API、静态资源和请求路由。
- `ros_bridge.py` 后台创建 ROS 节点，订阅 `/map`、`/scan`、`/odom`、`/plan`、`/tf` 等，并把数据整理到 `StateStore`。
- `process_manager.py` 启动/停止建图和导航 launch，读取运行日志。
- 前端页面轮询后端 API，显示地图、雷达、路径、机器人状态、运行日志，并提供建图/导航/配置编辑操作。

开发入口：

- 新增 HTTP API：`server/server_app.py`。
- 新增 ROS 状态订阅或控制命令：`server/ros_bridge.py`。
- 修改启动/清理逻辑：`server/process_manager.py`。
- 修改页面和交互：`server/web/`。

### 3.10 仿真模块

相关文件：

- `sim/README.md`
- `sim/launch/map_sim.launch.py`
- `sim/launch/nav_sim.launch.py`
- `sim/launch/path_plan_sim.launch.py`
- `sim/scripts/sim_chassis_sensor_pub.py`
- `sim/scripts/sim_gazebo_drive.py`
- `sim/scripts/gz_topic_adapter.py`
- `sim/scripts/sim_plan_visualizer.py`
- `sim/scripts/sim_send_goal.py`
- `sim/gazebo/`
- `sim/config/`
- `sim/rviz/path_plan_sim.rviz`

原理：

- 提供 Gazebo/RViz 环境，用于不依赖实车验证建图、导航和路径规划。
- 仿真脚本负责发布模拟底盘传感器、桥接 Gazebo 话题、发送目标点和显示规划结果。

开发入口：

- 仿真使用说明：`sim/README.md`。
- Gazebo 世界和模型：`sim/gazebo/`。
- 仿真 launch：`sim/launch/`。

## 4. 配置文件索引

- `config/base_control.yaml`：底盘、键盘和 HID 摇杆速度档位参数。
- `config/joy.yaml`：HID 摇杆设备、轴向、死区、饱和区和两档分界线。
- `config/lidar.yaml`：左右 FREE 雷达 IP、角度、频率、滤波、时间戳策略和融合参数。
- `config/imu.yaml`：DM-IMU 串口、零偏、发布参数。
- `config/ekf.yaml`：robot_localization EKF 融合参数。
- `config/slam_toolbox_map.yaml`：建图模式 SLAM 参数。
- `config/slam_toolbox_nav.yaml`：定位模式 SLAM 参数和默认地图加载参数。
- `config/path_plan.yaml`：路径规划参数。
- `config/nav.yaml`：路径跟踪控制参数。
- `config/fastdds_profiles.xml`：Fast DDS 配置，用于减少实机 DDS 兼容问题。

## 5. 构建与安装

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select finav
source install/setup.bash
```

如果同一工作区存在同名包，需要限制构建路径：

```bash
colcon build --base-paths src/finav --packages-select finav
source install/setup.bash
```

CMake 安装内容：

- `launch/`、`config/`、`rviz/`、`urdf/`、`maps/` 安装到 share 目录。
- 控制、IMU、工具、仿真脚本安装为 `ros2 run finav ...` 可执行程序。
- `third_party/free_lidar` 被编译为 `free_lidar_node`。
- `third_party/dm_imu/dm_imu_modules` 安装到 `lib/finav/dm_imu_modules`。

## 6. 推荐新人上手路径

1. 阅读本文，先理解整体数据流。
2. 阅读 `README.md`，了解 Web 后台接口和页面工作流。
3. 根据职责选择模块：
   - 底盘：`scripts/control/base_control.py`、`config/base_control.yaml`。
   - 雷达：`third_party/free_lidar/`、`src/rosnode/scan_fusion_node.cpp`、`config/lidar.yaml`。
   - IMU/EKF：`scripts/imu/dm_imu_publisher.py`、`config/imu.yaml`、`config/ekf.yaml`。
   - 建图定位：`launch/sub/slam_toolbox.launch.py`、`config/slam_toolbox_*.yaml`。
   - 路径规划：`scripts/control/path_plan.py`、`config/path_plan.yaml`。
   - 路径跟踪：`scripts/control/nav_control.py`、`config/nav.yaml`。
   - Web：`server/`、`server/web/`。
   - 仿真：`sim/`。
4. 先在仿真或单模块环境验证，再接入完整实机 launch。
5. 修改任何传感器坐标或时间戳相关逻辑后，必须同时检查 `/tf`、`/scan`、`/odom` 和 RViz 显示。

## 7. 常见问题与排查入口

### 7.1 雷达连接失败

检查：

```bash
ping 192.168.1.111
ip -br addr
ip route
```

上位机与两台雷达必须在同一网段。雷达直连网口不要配置默认网关，避免影响远程连接。

### 7.2 RViz 看不到 `/scan`

检查：

```bash
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic hz /scan_left --qos-reliability best_effort
ros2 topic hz /scan_right --qos-reliability best_effort
```

RViz 的 LaserScan Display 需要：

- `Topic = /scan`
- `Reliability Policy = Best Effort`
- Fixed Frame 与 TF 树匹配。完整链路优先用 `base_link` 或 `map`；只看原始雷达时用 `laser_left_frame` 或 `laser_right_frame`。

### 7.3 RViz 报雷达 Frame 不存在

`/scan_left`、`/scan_right` 或 `/scan` 的 `header.frame_id` 不等于 TF 树里已经存在对应坐标系。需要：

- 启动 `launch/sub/robot_model.launch.py` 发布 URDF TF；或
- RViz Fixed Frame 手动设为对应雷达 frame；或
- 临时用 `static_transform_publisher` 发布测试 TF。

### 7.4 墙面在雷达点云中呈分段斜线

按顺序排查：

1. 只启动雷达，车静止，分别看 `/scan_left`、`/scan_right` 和融合 `/scan`。
2. 若原始数据静止仍分段，检查点序、角度范围、`is_reverse_postion`、厂商滤波和雷达硬件。
3. 若原始正常、融合异常，检查 `laser_left_frame` / `laser_right_frame` 的安装角和 `scan_fusion_node` 参数。
4. 若静止正常、运动后分段，优先考虑运动畸变、时间戳、TF 延迟和里程计质量。
5. 排查原始点云时关闭滤波：`filter_switch: 0`、`single_filter_enable: false`。

### 7.5 SLAM 丢弃 scan

典型日志：

```text
Message Filter dropping message ... timestamp ... earlier than all the data in the transform cache
```

检查：

- `/scan.header.stamp` 是否过早。当前融合节点会用发布时刻作为 `/scan` 时间戳，若仍报错应继续查原始 scan 时间、TF 发布延迟和是否同时启动了多个雷达 launch。
- `use_recv_time_stamp` 是否开启。
- `robot_state_publisher`、EKF、SLAM 启动顺序。
- `slam_toolbox` 的 `transform_timeout` 和 TF 缓存。

### 7.6 导航不动或规划失败

检查：

```bash
ros2 topic echo /goal_pose --once
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
```

常见原因：

- 没有 `map -> odom -> base_link` TF。
- `/map` 未发布或地图 QoS 不匹配。
- 目标点在障碍/未知区域。
- `path_plan.yaml` 中车体尺寸、安全边距或未知区域策略过保守。

## 8. 文件速查表

| 领域 | 关键文件 |
| --- | --- |
| 项目构建 | `CMakeLists.txt`, `package.xml` |
| 一键启动 | `start_finav.sh`, `base_drive.sh` |
| 建图 launch | `launch/map.launch.py`, `launch/sub/slam_toolbox.launch.py` |
| 导航 launch | `launch/nav.launch.py` |
| 雷达 | `third_party/free_lidar/`, `src/rosnode/scan_fusion_node.cpp`, `launch/sub/lidar.launch.py`, `config/lidar.yaml` |
| IMU | `scripts/imu/dm_imu_publisher.py`, `third_party/dm_imu/`, `config/imu.yaml` |
| EKF | `launch/sub/ekf.launch.py`, `config/ekf.yaml` |
| 机器人模型 | `urdf/whillcar.urdf`, `launch/sub/robot_model.launch.py` |
| 底盘控制 | `scripts/control/base_control.py`, `config/base_control.yaml` | 故障检测 `/base_fault` |
| 摇杆/键盘 | `scripts/control/joy_control.py`, `launch/sub/joy.launch.py`, `config/joy.yaml`, `config/base_control.yaml`, `scripts/control/base_control_router.py` |
| 路径规划 | `scripts/control/path_plan.py`, `config/path_plan.yaml` |
| 路径跟踪 | `scripts/control/nav_control.py`, `config/nav.yaml` |
| Web 后台 | `server/`, `server/web/` |
| 地图 | `maps/`, `scripts/tool/save_map.sh`, `scripts/tool/manual_save_map.py` |
| RViz | `rviz/mapping.rviz`, `rviz/navigation.rviz` |
| 仿真 | `sim/` |
