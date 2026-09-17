# Finav 新人入门

Finav 是 ROS 2 Humble 实机建图与导航项目。先按 [README](../README.md#快速入口) 构建并启动，再按任务阅读下面三份文档。

| 需要做什么 | 文档 |
| --- | --- |
| 部署、启停、看日志、排查故障 | [服务部署](服务部署.md) |
| 接入导航、理解话题与路线行为 | [导航接口](navigation_topics.md) |
| 新机器准备驱动和设备配置 | [驱动与依赖](驱动与依赖清单.md) |

## 两个仓库

```text
nav_workspace/src/
  base_control/  # 底盘、HID/STM32 手柄、速度仲裁、底盘服务
  finav/         # 雷达、IMU、定位、建图、导航、Web
```

- [Flowhist/base_control](https://github.com/Flowhist/base_control)：`main` 分支，可独立构建运行。
- [Flowhist/nav-2026](https://github.com/Flowhist/nav-2026)：拆分集成在 `dev/finav-pro` 分支。
- `finav` 依赖 `base_control`，底盘不依赖导航。底层开发说明见 [底盘 README](../../base_control/README.md)。
- ROS/Python 包名使用 `base_control`，systemd 单元名使用 `base-control.service`。

## 系统怎样工作

```text
双雷达 -> /scan -> SLAM Toolbox -> /map、map->odom
编码器 + IMU -> EKF -> /odom、odom->base_link
目标点/已保存路线 -> 任务管理 -> 路径规划 -> /plan
/plan -> 路径跟踪 -> /nav_cmd_vel
手柄 / Web / 导航 -> 底盘仲裁 -> /cmd_vel -> 电机
```

建图与导航按需启动，传感器和定位节点随对应服务启停。底盘与 Web 独立运行，重启 Web 不影响手柄控制。

雷达对下游统一提供 `/scan`。EKF 融合编码器前向/角速度与 IMU Z 轴角速度，不融合 IMU 绝对 yaw。URDF 提供车体到雷达、IMU 的固定变换。

## 第一次操作

1. 按 README 安装服务，确认设备权限、急停状态和现场测试条件。
2. 启动 `finav.target`，打开 `http://<机器人IP>:8010`，先查看底盘和手柄状态。
3. 建图页开始建图，完成后保存地图并停止建图。
4. 地图页编辑地点、禁行区和开放路线。
5. 导航页选择地图、启动导航，确认定位后下发目标点或路线。
6. 测试结束执行 `systemctl --user stop finav.target`。

地图保存在 `maps/`，地图栅格、SLAM 定位数据和编辑数据应一起备份。`<地图名>.editor.yaml` 保存禁行区及路线，`.locations.yaml` 保存地点；地图默认被 Git 忽略，推送代码不会备份地图。

## 改哪里

以下路径相对 `finav/`；底盘路径指向同级仓库。

| 模块 | 代码入口 | 配置 |
| --- | --- | --- |
| 双雷达与融合 | `third_party/hinson_he_lidar/`、`src/rosnode/scan_fusion_node.cpp` | `config/lidar.yaml` |
| IMU | `scripts/imu/dm_imu_publisher.py` | `config/imu.yaml` |
| EKF / SLAM | `launch/sub/`、`launch/map.launch.py`、`launch/nav.launch.py` | `config/ekf.yaml`、`config/slam_toolbox_*.yaml` |
| 任务与路线 | `scripts/control/nav_task_manager.py`、`route_tasks.py` | `config/nav_task.yaml` |
| 规划与禁行区 | `scripts/control/nav_path_plan.py`、`planning_constraints.py` | `config/path_plan.yaml` |
| 路径跟踪 | `scripts/control/nav_control.py` | `config/nav.yaml` |
| 地图编辑 | `scripts/map_location/editor_*.py`、`server/web/map-editor.js` | 地图目录的 `.editor.yaml` |
| Web 与服务 | `server/server_app.py`、`ros_bridge.py`、`systemd_runtime.py`、`scripts/service/` | `~/.local/state/finav/` |
| 底盘 / 手柄 | `../base_control/base_control/` | `../base_control/config/` |
| 安装位姿 / RViz | `urdf/whillcar.urdf`、`rviz/` | URDF 与 `.rviz` 文件 |

## 开发与验证

优先修改对应仓库，保留 ROS 接口；新增运动来源应进入仲裁器，不直接向 `/cmd_vel` 发指令。构建时显式指定包路径，避免扫描到重复包。

底盘无硬件检查可在工作区执行：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --base-paths src/base_control --packages-select base_control
```

目前导航仓库没有可直接运行的 `tests/` 测试集。构建不代替实机验收：运动改动需验证松杆停车、持续指令、接管及急停；服务改动需验证独立重启、模式切换和开机行为。
