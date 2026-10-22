# Finav

Finav 是一个基于 ROS 2 Humble 的实机导航项目，包含 Hinson HE-3051 激光雷达、DM-IMU、EKF 融合、SLAM Toolbox 建图/定位、自研路径规划与路径跟踪、Web 调试后台和 Gazebo/RViz 仿真环境。WHILL 底盘、手柄及控制仲裁归同级独立仓库 `base_control`；仓库分工见 [新人入门](Doc/项目Wiki.md)。

新开发者请先阅读项目 Wiki：

- [Doc/项目Wiki.md](Doc/项目Wiki.md)

文档按任务分为四份：[新人入门](Doc/项目Wiki.md)、[服务部署](Doc/服务部署.md)、[导航接口](Doc/navigation_topics.md)、[驱动与依赖](Doc/驱动与依赖清单.md)。先跑通服务，再查对应模块的代码与配置。

## 快速入口

在工作区根目录执行构建和服务安装；`src/` 下需要同时有 `finav` 和 `base_control` 两个仓库：

```bash
cd /home/embotic/nav_workspace
source /opt/ros/humble/setup.bash
colcon build --base-paths src/base_control src/finav --packages-select base_control finav --symlink-install
source install/local_setup.bash
python3 src/finav/scripts/service/install_services.py
```

上述构建已限定包路径，避免工作区存在同名包时冲突。安装脚本只生成当前用户的 systemd 单元，不启动服务、不启用自启动；不要用 sudo 执行安装脚本。仓库位置改变后需重新构建、安装。

## 服务启动与管理

所有 `systemctl --user` 命令均由安装服务的机器人用户执行，不加 sudo。启动底盘前确认现场安全、手柄回中。

| 服务单元 | 职责 |
| --- | --- |
| `base-control.service` | 底盘驱动、手柄和速度仲裁 |
| `finav-web.service` | 网页后台和 ROS 状态桥接 |
| `finav-mapping.service` | 建图链路 |
| `finav-navigation.service` | 导航链路 |
| `finav.target` | 统一管理上述服务；启动时只拉起底盘和 Web |

### 日常启动与停止

```bash
# 启动底盘和网页
systemctl --user start finav.target

# 也可以分别启动
systemctl --user start base-control.service
systemctl --user start finav-web.service

# 只重启网页，不影响手柄控制
systemctl --user restart finav-web.service

# 停止全部服务，包括已启动的建图和导航
systemctl --user stop finav.target
```

浏览器访问 `http://<机器人IP>:8010`，机器人本机可访问 `http://localhost:8010`。在网页中启动、停止建图或导航；导航前先选择地图，建图与导航不能同时运行。

开发调试阶段默认关闭口令登录，浏览器可直接访问。可选访问控制设置见 [Web 访问](Doc/服务部署.md#web-访问)。

单独停止服务：

```bash
systemctl --user stop finav-mapping.service
systemctl --user stop finav-navigation.service
systemctl --user stop finav-web.service
systemctl --user stop base-control.service
```

停止或重启底盘会同时停止建图、导航；单独停止或重启 Web 不会停止这些服务。整套重启用 `systemctl --user restart finav.target`，只恢复底盘和 Web，不恢复之前的导航任务。

### 状态和日志

```bash
systemctl --user status base-control.service finav-web.service finav-mapping.service finav-navigation.service --no-pager

# 节点输出：底盘日志同时包含手柄和仲裁器
tail -n 50 -F ~/.local/state/finav/base.log
tail -n 50 -F ~/.local/state/finav/web.log
tail -n 50 -F ~/.local/state/finav/mapping.log
tail -n 50 -F ~/.local/state/finav/navigation.log

# 服务启动器错误
journalctl --user -u base-control.service -u finav-web.service -n 100 --no-pager
```

日志窗口按 `Ctrl+C` 只退出查看，不会停止服务。`active` 仅表示服务进程在运行，设备连接和定位是否就绪仍需查看日志或网页状态。

### 开机自启动

```bash
# 无需登录即可运行用户服务；在机器人用户的终端执行
sudo loginctl enable-linger "$USER"
systemctl --user enable finav.target

# 检查是否已启用
systemctl --user is-enabled finav.target

# 取消下次开机自启动，不停止当前服务
systemctl --user disable finav.target
```

开机仅启动底盘和 Web，建图、导航仍由网页按需启动。`enable` 不会立即启动服务；需要立即启动时另执行 `systemctl --user start finav.target`。

底盘配置位于 `base_control/config/`，导航配置位于 `finav/config/`；保存后按网页提示重启对应服务。详细部署、参数和回退说明见 [服务部署](Doc/服务部署.md)。已安装服务的机器不要再运行旧 `start_finav.sh` 或另起同一套 ROS 节点。

## 项目目录

常用目录：

- `launch/`：实机建图、导航和子模块启动文件。
- `config/`：雷达、IMU、EKF、SLAM、规划和导航控制参数。
- `scripts/control/`：导航任务、路径规划、路径跟踪。
- `../base_control/`：独立底盘与手柄仓库，包括驱动、仲裁和底层配置。
- `scripts/imu/`：DM-IMU ROS 2 发布节点。
- `scripts/map_location/`：地图地点文件读取和 RViz 地点可视化。
- `third_party/`：Hinson HE-3051 雷达和 DM-IMU 厂商代码整理区。
- `server/`：Web 调试后台。
- `sim/`：仿真 launch、脚本、Gazebo 模型和世界。
- `maps/`：已保存地图。
- `Doc/`：项目级文档。

---

# Finav Web Server

`server/` 是 Finav 建图 / 导航项目的轻量网页调试后台。
它同时承担 4 件事：

1. 把 ROS2 运行状态整理成网页可持续推送的数据。
2. 把网页上的控制操作转成 ROS 指令或启动 / 清理脚本。
3. 提供地图预览、配置文件编辑、运行日志读取等 HTTP 接口。
4. 维护一份线程安全的全局状态，供前端页面统一消费。

当前实现以“单进程 Python HTTP 服务 + 后台 ROS bridge 线程 + 原生 HTML/CSS/JS 前端”为主，适合内网调试和实机联调，不是面向公网的通用服务。

## 目录结构

- `run_server.py`
  - server 启动入口。
- `server_app.py`
  - HTTP 服务、路由分发、静态文件服务。
- `ros_bridge.py`
  - ROS2 桥接层，订阅地图、规划约束、雷达、位姿、导航任务和底盘仲裁状态，并处理网页控制命令。
- `process_manager.py`
  - 未安装服务时的进程管理及共用日志、重定位辅助功能。
- `systemd_runtime.py`
  - 已安装服务时的运行管理，负责服务启停、状态查询和模式切换。
- `state_store.py`
  - 线程安全状态仓库，保存 `status`、`scene`、事件历史。
- `map_utils.py`
  - 复用地图编辑器的地图解析及共享地点读取函数，提供 Web 预览数据。
- `web_support.py`
  - HTTP 连接限制、身份验证、请求校验与配置原子保存。
- `web/index.html`
  - 页面结构。
- `web/styles.css`
  - 样式。
- `web/app-core.js`
  - 前端基础状态、地图基础渲染工具。
- `web/app-api.js`
  - API 请求、超时与 HTTP/ROS/画面流连接状态。
- `web/app-dock.js`
  - 底栏状态 / 事件 / 日志区域逻辑。
- `web/app-pages.js`
  - 实时场景流、地图与导航地点/路线列表、配置编辑器、服务控制。
- `web/app-manual-control.js`
  - 网页虚拟摇杆、键盘控制、回中与停止逻辑。
- `web/map-editor.js`
  - 地点、禁行区、路线编辑与地图导出；后端逻辑位于 `scripts/map_location/editor_*.py`。
- `web/app.js`
  - 前端主交互绑定、导航拖拽、页面切换、初始化入口。
- `runtime/`
  - 仅旧进程模式使用；服务模式日志位于 `~/.local/state/finav/`。

## 页面与工作流

### 1. 建图页

- 手动控制面板提供共用的 1–5 档切换。Web 经 `/handle_control/set_gear` 请求现有手柄节点写十进制寄存器 23，再读取寄存器 3 确认；成功后 `/handle/gear` 同步更新 Web 和控制链路。协议规定 MCU 同步刷新触摸屏，屏幕显示仍需实机验收。档位是设备运行状态，不写入导航 YAML。
- 左侧显示实时 `/map` 和 `/scan`。
- 右侧包含：
  - 建图启动 / 结束
  - 地图保存
- “开始建图”会调用：
  - `POST /api/runtime/mapping/start`
  - 服务模式启动 `finav-mapping.service`，由服务运行 `map.launch.py`
- “结束建图”或运行期间确认离开建图页会调用（不会自动保存地图）：
  - `POST /api/runtime/mapping/stop`
  - 服务模式停止 `finav-mapping.service` 及其子进程

### 2. 导航页

- “导航速度”可设置直行上限（0.05–0.60 m/s）、原地转向及行进转向上限（1–28.6 °/s），对应 `config/nav.yaml` 的 `nav_control.ros__parameters` 下 `linear_speed_mps`、`angular_speed_degps` 和 `drive_angular_speed_degps`。保存保留其他字段与注释；角速范围不超过当前底盘 0.5 rad/s 硬限。运行中通过 `/nav_control/set_speed` 一起应用；接口离线或失败时明确显示“已保存、当前导航未确认应用”，下次启动读取 YAML。最终输出仍受手柄档位比例、路线限速和底盘硬限约束。
- 新增设置接口使用 `rcl_interfaces/srv/SetParametersAtomically`：导航接受以上三个速度参数，手柄接受一个 `gear` 整数参数；需重启对应节点加载新接口。Web 的配置 GET/POST 接口为 `/api/nav/speed`，共用档位 POST 接口为 `/api/control/gear`。正常页面无新增周期轮询。
- 左侧显示实时地图、雷达和规划路径。
- 导航默认固定使用保存的 YAML/PGM，不再周期性重建底图；雷达、位姿和路径仍实时更新。
  `config/slam_toolbox_nav.yaml` 中 `freeze_navigation_map: false` 可恢复原有地图更新，重启导航后生效。
  STB 定位仍使用原有位姿图和扫描数据，详见 [固定地图定位](Doc/固定地图定位.md)。
- 右侧包含：
  - 导航启动 / 结束
  - 设置初始位姿
  - 设置目标点、按零散地点名字正常导航，或按路线终点名字发起巡线
  - 已保存开放路线的启动、暂停、继续和取消
  - 重定位
  - 停止 / 取消
- “开始导航”会调用：
  - `POST /api/runtime/navigation/start`
  - 服务模式启动 `finav-navigation.service`，由服务运行 `nav.launch.py`
- “结束导航”或运行期间确认离开导航页会调用：
  - `POST /api/runtime/navigation/stop`
  - 服务模式停止 `finav-navigation.service` 及其子进程

### 3. 地图页

- 从 `maps/` 目录读取地图元数据和 `.pgm`，在前端 canvas 中展示。
- 支持地点、禁行区、开放路线编辑，路线平滑、地图导出及删除。
- 地图页进入“编辑”后选择“画笔”，支持占据（黑）、可通行（白）、未知（灰），方形笔刷边长为 1–40 格，设为 N 时覆盖 N×N 格，支持撤销、保存和取消。仅在激活时绑定绘画事件，按需重绘；单次最多修改 100000 格，撤销最多保留 20 笔且限制历史栅格数量。
- 栅格保存需先停止建图、导航及重定位，会校验源地图版本并原子替换 PGM，上一版保存在同目录的 `<图像文件名>.brush-backup`。未修改的像素保持原值。此功能不修改 `.posegraph` / `.data`；SLAM Toolbox 从位姿图重建的地图不会自动包含这些笔画。
- 地点、禁行区和路线统一保存在 `maps/<地图名>/<地图名>.editor.yaml`，使用 revision 检测保存冲突并原子写入。没有编辑文档时兼容读取旧 `.locations.yaml`，之后不再双写地点文件。
- 删除地图前必须停止建图、导航和重定位，并等待存图结束。
- 路线执行行为见 [导航接口](Doc/navigation_topics.md#开放路线)。

### 4. 配置文件页

- 从导航仓库 `config/` 和底盘仓库的配置目录读取 `.yaml/.yml` 文件列表。
- 支持网页直接查看、编辑、保存。
- 采用纯文本编辑和轻量 YAML 高亮；后端校验 YAML 语法、非空映射及 `ros__parameters` 映射结构，按内容版本检查冲突后原子保存原始文本。
- 保存后显示配置对应的生效提示；底盘配置通过顶栏“⋯”菜单中的“重启底盘控制”应用，该操作会停止当前建图或导航，Web 保持运行。
- 顶栏“⋯”菜单还提供整套重启和关闭 Finav 服务；整套重启只恢复底盘与 Web，“关闭”不会关闭 Jetson 系统。

### 5. 运行状态中心

- 常驻于建图 / 导航页底部，展开后覆盖大部分工作区，不触发页面切换或任务清理。
- 汇总建图 / 导航就绪条件、地图 / 雷达 / 里程计 / TF 新鲜度与频率、机器人位姿、目标、路径、手柄链路、运行进程和日志。
- 展开期间暂停被遮挡的实时画面流，状态数据仍按低频更新，以减少无效传输和渲染。
- 服务区分启动中、已就绪、未就绪、停止中和失败；启动超过 30 秒仍缺少关键数据时显示未就绪，无目标时不要求存在路径。
- 事件支持级别筛选和保留条数选择；重启 Web 后自动恢复事件读取，历史截断时显示提示。

### 6. 网页手动控制

- 提供虚拟摇杆和键盘控制，经 `/api/control/manual` 转为 `/web_cmd_vel`，由底盘仲裁器处理。
- 控制量按手柄档位缩放；档位不在线时拒绝非零输入，手柄抢停后的锁定状态要求网页控制回中。
- 页面切换、失焦或隐藏时执行停止逻辑。

## 后端数据流

### 状态流

`ros_bridge.py` 后台启动一个 ROS2 节点 `web_control_server`：

- 订阅 `/map`
  - 对栅格内容计算签名，仅在内容变化时更新 `scene.map` 和递增 `map_version`
  - 栅格以 `int8 + Base64` 紧凑传输
- 订阅 `/planning_map`
  - 更新规划约束覆盖层，使用独立的 `planning_map_version`
- 订阅 `/scan`
  - 仅在网页存在可见实时画面时处理
  - 距离压缩为毫米级 `uint16 + Base64`，浏览器侧完成极坐标投影
- 订阅 `/odom`
  - 更新机器人里程计位姿、速度
- 订阅 `/plan`
  - 更新路径点数、长度、路径点数组及 `plan_version`
- 订阅 `/nav_status`
  - 更新导航任务类型、阶段、目标及原因
- 订阅 `/tf`
  - 统计 `map->odom`、`odom->base_link` 频率
- 订阅 `/js_state`、`/js_cmd_vel`、`/handle/gear`、`/base_control_router/status`
  - 更新手柄在线、输入、档位和网页控制锁定状态
- 订阅 `/nav_voice_bridge/status`
  - 更新地点/路线终点名称匹配的请求反馈；执行状态查看 `/nav_status`
- 定时器：
  - `0.02s` 处理网页命令队列
  - `0.25s` 刷新 `map` 坐标系下机器人位姿
  - `1.0s` 汇总频率统计

这些数据最终进入 `StateStore`：

- `status`
  - 机器人状态
  - 系统状态
  - 手柄控制状态
  - runtime 状态
  - 实时画面传输状态
- `scene`
  - 地图
  - 雷达点
  - 路径
  - 机器人位姿
  - 初始位姿 / 目标点
- `history`
  - 后端事件流

### 控制流

前端通过 HTTP 调接口：

- 实时画面
  - `/api/stream`（SSE；建图 10 Hz，导航 15 Hz）
- 导航
  - `/api/nav/initialpose`
  - `/api/nav/goal`
  - `/api/nav/cancel`
  - `/api/nav/route`、`/api/nav/location`、`/api/nav/relocate`
- 手动控制
  - `/api/control/manual`
- 建图存图
  - `/api/map/save`
- 运行时
  - `/api/runtime/*`
- 配置
  - `/api/configs*`
- 整套服务控制
  - `/api/system/*`

`server_app.py` 接口收到请求后：

- 轻量类请求直接读文件 / 返回状态。
- ROS 控制类请求进入 `RosBridge.command(...)`。
- launch / cleanup 类请求进入 `RuntimeManager`。

## 运行时管理

服务模式使用 `systemd_runtime.py` 管理独立服务，日志写入 `~/.local/state/finav/`。Web 通过 systemd 查询运行状态，重启 Web 不会接管或终止其他服务进程。

运行后端默认根据 `~/.config/finav/services.json` 是否存在选择，也可用 `FINAV_RUNTIME_BACKEND` 指定；已安装服务时拒绝启用旧进程模式。服务模式重启 Web 会结束其一次性重定位辅助进程。

未安装服务时，旧进程模式由 `process_manager.py` 负责：

- 启动建图 / 导航 launch
- 维护 `mapping` / `navigation` 进程状态
- 把 stdout/stderr 写入 `server/runtime/*.log`
- 终止时调用清理脚本
- 提供日志 tail 读取与清空接口

当前行为：

- 启动某一模式前，会先停止另一模式，避免建图 / 导航并存。
- 日志读取已做两层优化：
  - 按文件尾部读取，不再每次轮询整文件
  - 按文件标识、纳秒修改时间、大小和行数缓存，并通过版本参数跳过未变化的传输与渲染
- 启动时会强制设置：
  - `LANG=C.UTF-8`
  - `LC_ALL=C.UTF-8`
  - `PYTHONIOENCODING=utf-8`

## HTTP API

### 基础状态

- `GET /api/health`
- `GET /api/status`
- `GET /api/stream?mode=mapping|navigation&map_version=<n>&planning_map_version=<n>&plan_version=<n>`
- `GET /api/scene?map_version=<n>&planning_map_version=<n>`
- `GET /api/history?since=<seq>&limit=<n>&session_id=<id>`（响应含 `session_id`、`latest_seq`、`reset`、`truncated`）

### 地图

- `GET /api/maps`
- `GET /api/maps/<name>`
- `POST /api/map/save` `{ name }`（建图期间受理，返回 `202` 与 `job`；通过 `/api/status` 的 `map_save` 查询完成/失败，超时为 120 秒）
- `GET /api/maps/<name>/locations`
- `POST /api/maps/<name>/locations` `{ locations, expected_revision }`（读接口返回 `revision`，保存统一进入编辑文档）
- `GET /api/maps/<name>/editor`
- `PUT /api/maps/<name>/editor` `{ expected_revision, document }`
- `POST /api/maps/<name>/routes/<route_id>/smooth` `{ expected_revision }`
- `POST /api/maps/<name>/export`
- `POST /api/maps/<name>/delete`（删除地图目录及其文件）

### 导航 / 控制

- `POST /api/nav/goal` `{ x, y, yaw_deg }`
- `POST /api/nav/initialpose` `{ x, y, yaw_deg }`
- `POST /api/nav/cancel`
- `POST /api/nav/route` `{ action, map_name, route_id }`，`action` 为 `start/pause/resume/cancel`
- `POST /api/nav/location` `{ name }`（命中当前地图路线的 `end_name` 时巡线，命中零散地点的 `name` 时正常导航）
- `POST /api/nav/relocate` `{ map_file }`
- `POST /api/control/manual` `{ linear, angular }`（归一化输入，非实际速度；零输入表示停止）

ROS 控制与存图接口返回后仍需查看状态和事件确认执行结果；存图通过后台脚本完成。

口令登录默认关闭，只有显式设置 `FINAV_WEB_AUTH=1` 才启用 Basic 身份验证。写请求使用 `application/json`，拒绝跨站来源；无效参数返回 4xx，ROS 离线或普通命令队列已满返回 503。手动输入保留最新值并在 300 毫秒后过期，停止/取消指令优先处理。

### 运行时

- `POST /api/runtime/mapping/start`
- `POST /api/runtime/mapping/stop`
- `POST /api/runtime/navigation/start` `{ map_file }`
- `POST /api/runtime/navigation/stop`
- `GET /api/runtime/logs/mapping?tail=<n>`
- `GET /api/runtime/logs/navigation?tail=<n>`
- `POST /api/runtime/logs/mapping/clear`
- `POST /api/runtime/logs/navigation/clear`
- `POST /api/runtime/<mode>/restart`，模式包括 `base`、`base_drive`、`handle`、`router`、`mapping`、`navigation`；服务模式下前四项都重启整个底盘服务

日志读取与清空还支持 `relocate`、`base`、`handle`、`router`、`base_drive`；服务模式下底盘相关名称映射到同一份 `base.log`。Web 日志通过部署文档中的文件或 journal 命令查看。

日志读取可附加上次响应的 `version` 参数，未变化时返回 `unchanged: true`。

### 配置文件

- `GET /api/configs`
- `GET /api/configs/<filename>`
- `POST /api/configs/<filename>` `{ content, expected_revision }`（读取时取得 `revision`；冲突返回 409，YAML/基本结构校验失败返回 422）

### Finav 服务

- `GET /api/system`
- `POST /api/system/shutdown`
- `POST /api/system/restart`

## 前端实现说明

当前前端是“无构建工具”的原生页面：

- `index.html`
  - 负责页面骨架。
- `styles.css`
  - 负责全部页面样式。
- `app-core.js`
  - 负责全局状态、基础工具、通用 canvas 渲染。
- `app-api.js`
  - 负责 HTTP 请求、错误解析与分通道连接状态。
- `app-dock.js`
  - 负责底栏状态 / 事件 / 运行日志。
- `app-pages.js`
  - 负责场景流、地图/地点/路线列表、配置页编辑与保存、运行服务控制。
- `app-manual-control.js`
  - 负责虚拟摇杆、键盘控制及停止事件。
- `map-editor.js`
  - 负责地图对象编辑、路线平滑及导出。
- `app.js`
  - 负责导航拖拽、页面切换、事件绑定、应用初始化。

脚本按 `app-core.js`、`app-api.js`、`app-dock.js`、`app-pages.js`、`app-manual-control.js`、`map-editor.js`、`app.js` 顺序加载，共用 `appState`；地图编辑器通过 `window.finavMapEditor` 暴露入口。

## 启动

已安装服务时，只启动网页后台：

```bash
systemctl --user start finav-web.service
```

同时启动底盘和网页：

```bash
systemctl --user start finav.target
```

浏览器访问 `http://<机器人IP>:8010`。完整管理命令见上方“服务启动与管理”。

仅在未安装服务的开发环境中，可从工作区根目录手动启动 Web：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 src/finav/server/run_server.py --host 0.0.0.0 --port 8010
```

## 调试建议

- 看 server 是否起起来：`/api/health`
- 看 ROS bridge 是否正常：`/api/status`
- 看实时场景是否推送：`/api/stream?mode=mapping`
- 看运行日志：
  - `~/.local/state/finav/mapping.log`
  - `~/.local/state/finav/navigation.log`
- 如果网页按钮已点但功能没起来：
  - 先看运行日志
  - 再看 `events` 里的 `start requested / cleanup started / process exited`

## 注意

- 这是内网调试工具；Basic 身份验证不加密 HTTP 传输，跨不可信网络访问时需使用 HTTPS 反向代理或安全隧道。
- `cancel_nav` 现在会同时急停并清掉当前导航目标 / 路径状态。
- 配置页会直接覆盖 `config/` 下文件，建议调参前保留原始备份。

## Web 离线验证

在工作区根目录运行，使用临时地图/配置与模拟服务，不启动 ROS 节点或操作硬件：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="src/base_control:src/finav/server:$PYTHONPATH" python3 -m unittest discover -s src/finav/server/tests -v
```
