# Finav Web Server

`server/` 是 Finav 建图 / 导航项目的轻量网页调试后台。
它同时承担 4 件事：

1. 把 ROS2 运行状态整理成网页可轮询的数据。
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
  - ROS2 桥接层，负责订阅 `/map`、`/scan`、`/odom`、`/plan`、`/tf`，并处理网页控制命令。
- `process_manager.py`
  - 建图 / 导航运行时管理，负责启动 `map.launch.py`、`nav.launch.py`，读取日志，调用清理脚本。
- `state_store.py`
  - 线程安全状态仓库，保存 `status`、`scene`、事件历史。
- `map_utils.py`
  - 读取 `maps/` 下的 `.yaml + .pgm`，供“已有地图预览”页面使用。
- `web/index.html`
  - 页面结构。
- `web/styles.css`
  - 样式。
- `web/app-core.js`
  - 前端基础状态、API 请求、地图基础渲染工具。
- `web/app-dock.js`
  - 底栏状态 / 事件 / 日志区域逻辑。
- `web/app-pages.js`
  - 页面数据加载、地图列表、配置编辑器。
- `web/app.js`
  - 前端主交互绑定、键控、导航拖拽、初始化入口。
- `runtime/`
  - `mapping.log`、`navigation.log` 运行日志输出目录。

## 页面与工作流

### 1. 建图页

- 左侧显示实时 `/map` 和 `/scan`。
- 右侧包含：
  - 建图启动 / 结束
  - 网页键控
  - 速度档位
  - 地图保存
- “开始建图”会调用：
  - `POST /api/runtime/mapping/start`
  - 后端实际执行 `ros2 launch finav map.launch.py`
- “结束建图”或离开建图页会调用：
  - `POST /api/runtime/mapping/stop`
  - 后端执行 `scripts/tool/clean_map.sh`

### 2. 导航页

- 左侧显示实时地图、雷达和规划路径。
- 右侧包含：
  - 导航启动 / 结束
  - 设置初始位姿
  - 设置目标点
  - 停止 / 取消
- “开始导航”会调用：
  - `POST /api/runtime/navigation/start`
  - 后端实际执行 `ros2 launch finav nav.launch.py`
- “结束导航”或离开导航页会调用：
  - `POST /api/runtime/navigation/stop`
  - 后端执行 `scripts/tool/clean_nav.sh`

### 3. 地图预览页

- 从 `maps/` 目录读取地图元数据和 `.pgm`，在前端 canvas 中展示。
- 页面只读，不显示底栏状态区。

### 4. 配置文件页

- 从 `config/` 读取 `.yaml/.yml` 文件列表。
- 支持网页直接查看、编辑、保存。
- 当前是“纯文本编辑 + 前端轻量 YAML 高亮”，未做 YAML 语法校验。

### 5. 底栏

- 左 1/3 为状态总览：
  - 机器人状态
  - 系统状态
  - 包含网页控制、摇杆控制等实时接管状态
- 右 2/3 为事件 / 日志区：
  - 事件流
  - 建图日志
  - 导航日志

## 后端数据流

### 状态流

`ros_bridge.py` 后台启动一个 ROS2 节点 `web_control_server`：

- 订阅 `/map`
  - 更新 `scene.map`
  - 递增 `map_version`
- 订阅 `/scan`
  - 把激光点投影到 `map` 坐标系
  - 更新 `scene.scan`
- 订阅 `/odom`
  - 更新机器人里程计位姿、速度
- 订阅 `/plan`
  - 更新路径点数、长度、路径点数组
- 订阅 `/tf`
  - 统计 `map->odom`、`odom->base_link` 频率
- 订阅 `/js_state`
  - 更新底栏里的摇杆控制激活状态
- 定时器：
  - `0.05s` 处理网页命令队列
  - `0.1s` 执行 teleop 超时刹停
  - `0.25s` 刷新 `map` 坐标系下机器人位姿
  - `1.0s` 汇总频率统计

这些数据最终进入 `StateStore`：

- `status`
  - 机器人状态
  - 系统状态
  - teleop 状态
  - runtime 状态
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

- teleop
  - `/api/teleop/cmd_vel`
  - `/api/teleop/stop`
- 导航
  - `/api/nav/initialpose`
  - `/api/nav/goal`
  - `/api/nav/cancel`
- 建图存图
  - `/api/map/save`
- 运行时
  - `/api/runtime/*`
- 配置
  - `/api/configs*`

`server_app.py` 接口收到请求后：

- 轻量类请求直接读文件 / 返回状态。
- ROS 控制类请求进入 `RosBridge.command(...)`。
- launch / cleanup 类请求进入 `RuntimeManager`。

## 运行时管理

`process_manager.py` 负责：

- 启动建图 / 导航 launch
- 维护 `mapping` / `navigation` 进程状态
- 把 stdout/stderr 写入 `server/runtime/*.log`
- 终止时调用清理脚本
- 提供日志 tail 读取与清空接口

当前行为：

- 启动某一模式前，会先停止另一模式，避免建图 / 导航并存。
- 日志读取已做两层优化：
  - 按文件尾部读取，不再每次轮询整文件
  - 按 `mtime + size + tail` 做简单缓存
- 启动时会强制设置：
  - `LANG=C.UTF-8`
  - `LC_ALL=C.UTF-8`
  - `PYTHONIOENCODING=utf-8`

## HTTP API

### 基础状态

- `GET /api/health`
- `GET /api/status`
- `GET /api/scene?map_version=<n>`
- `GET /api/history?since=<seq>&limit=<n>`

### 地图

- `GET /api/maps`
- `GET /api/maps/<name>`
- `POST /api/map/save` `{ name }`

### 导航 / 控制

- `POST /api/nav/goal` `{ x, y, yaw_deg }`
- `POST /api/nav/initialpose` `{ x, y, yaw_deg }`
- `POST /api/nav/cancel`
- `POST /api/teleop/cmd_vel` `{ linear_x, angular_z, timeout_ms }`
- `POST /api/teleop/stop`

### 运行时

- `POST /api/runtime/mapping/start`
- `POST /api/runtime/mapping/stop`
- `POST /api/runtime/navigation/start`
- `POST /api/runtime/navigation/stop`
- `GET /api/runtime/logs/mapping?tail=<n>`
- `GET /api/runtime/logs/navigation?tail=<n>`
- `POST /api/runtime/logs/mapping/clear`
- `POST /api/runtime/logs/navigation/clear`

### 配置文件

- `GET /api/configs`
- `GET /api/configs/<filename>`
- `POST /api/configs/<filename>` `{ content }`

## 前端实现说明

当前前端是“无构建工具”的原生页面：

- `index.html`
  - 负责页面骨架。
- `styles.css`
  - 负责全部页面样式。
- `app-core.js`
  - 负责全局状态、基础工具、API 请求、通用 canvas 渲染。
- `app-dock.js`
  - 负责底栏状态 / 事件 / 运行日志。
- `app-pages.js`
  - 负责场景轮询、地图预览、配置页编辑与保存。
- `app.js`
  - 负责 teleop、导航拖拽、页面切换、事件绑定、应用初始化。

### 当前前端结构评价

- 优点
  - 部署简单，没有前端构建链。
  - 已按功能边界做了一次轻量拆分，阅读成本比单文件低。
  - 浏览器兼容要求低。
  - 调试路径短，适合现场联调。
- 问题
  - 目前仍是全局函数 + 顺序脚本加载，模块边界主要靠约定。
  - 地图渲染、配置编辑、运行时控制虽然已分文件，但仍共享同一份全局状态。
  - 后续继续加页面逻辑，维护成本会明显上升。

### 是否需要拆前端代码

有必要，但当前先停在“原生多文件拆分”这一层更合适。

原因：

- 当前前端仍然没有 bundler。
- 对这个项目来说，先用原生多文件把职责切开，已经能明显提升可读性。
- 如果继续上 ES module / Vite，需要一起梳理静态资源路径、模块依赖和部署方式，改动面会更大。

当前已经拆到这个粒度：

- `web/app-core.js`
  - 全局状态、基础工具函数、API 请求、地图基础渲染。
- `web/app-dock.js`
  - 状态栏、事件流、运行日志。
- `web/app-pages.js`
  - 地图预览、配置文件页、场景轮询。
- `web/app.js`
  - teleop、导航拖拽、绑定、轮询启动。

如果后面前端继续扩张，再考虑：

- 改成浏览器原生 ES module
- 或者再进一步引入 bundler

## 当前架构是否合理

对当前项目阶段来说，整体是合理的。

### 合理点

- Python HTTP 服务足够轻，部署简单。
- ROS2 bridge 独立线程，和 HTTP 路由职责分离。
- `StateStore` 把“实时状态”和“事件流”统一收口，前端消费简单。
- `RuntimeManager` 把 launch / cleanup / log 从 `server_app.py` 中抽离出来，方向是对的。
- 地图、配置、日志都走显式 API，而不是前端直接碰文件系统，边界清晰。

### 目前主要风险
1. `scene` 深拷贝成本高
   - `/api/scene` 每 350ms 轮询一次。
   - 当前 `StateStore.snapshot_scene()` 仍会深拷贝 `scan.points`、`plan.points_xy` 等大数组。
   - 当地图、雷达点很多时，这会成为 CPU 和内存复制开销。

2. `/map` 数据体积大
   - 当前 live map 直接把 occupancy grid 全量放进内存并传给前端。
   - 虽然通过 `map_version` 避免了每次都重发地图，但首次下发和地图变化时仍然很重。

4. 缺少鉴权
   - 内网调试可接受，但只要被其他机器访问，就能直接发 `/cmd_vel`、起停 launch、改配置文件。

5. 配置编辑缺少语法校验
   - 当前是“保存即写盘”，如果用户改坏 YAML，后续 launch 会直接失败。

6. 前端 monolith 风险
   - `app.js` 单文件体量已到维护拐点。

## 可继续优化的地方

1. 给 `/api/scene` 做更强的差量返回
   - 当前只对 map 做了 `map_version` 优化。
   - 扫描点、路径点也可以做版本号或时间戳 gating。

2. 给日志接口加 `since_mtime` / `etag`
   - 前端如果发现日志没更新，可以直接跳过渲染。

3. 配置文件保存前做 YAML 解析校验
   - 至少提示“语法错误，不保存”。

4. 给 `RuntimeManager` 增加启动成功判定
   - 现在“start”更多表示命令已发出。
   - 更稳的方式是结合 `/map`、`/plan`、关键进程存活状态给出“已启动 / 启动失败”。

5. 为事件流增加级别过滤或保留上限控制
   - 当前默认最多 2000 条，前端显示 240 条。

## 启动

只起网页后台时，在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
source /home/flowhist/workspace/final_ws/install/local_setup.bash
python3 server/run_server.py --host 0.0.0.0 --port 8010
```

如果需要把底盘、摇杆与网页后台一起拉起，直接在项目根目录执行：

```bash
bash start_finav.sh
```

浏览器访问：

```text
http://<你的IP>:8010
```

## 调试建议

- 看 server 是否起起来：`/api/health`
- 看 ROS bridge 是否正常：`/api/status`
- 看 live map 是否刷新：`/api/scene`
- 看运行日志：
  - `server/runtime/mapping.log`
  - `server/runtime/navigation.log`
- 如果网页按钮已点但功能没起来：
  - 先看运行日志
  - 再看 `events` 里的 `start requested / cleanup started / process exited`

## 注意

- 这是内网调试工具，当前未加鉴权，不应直接暴露公网。
- `cancel_nav` 现在会同时急停并清掉当前导航目标 / 路径状态。
- 配置页会直接覆盖 `config/` 下文件，建议调参前保留原始备份。
