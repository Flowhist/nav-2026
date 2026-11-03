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
- `realtime.py`、`web/app-realtime.js`
  - 唯一实时传输链路：同端口 WebSocket、主题订阅、二进制附件、确认流控和断线重连。
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
- `web/app-perception.js`、`perception_view.py`
  - 独立俯视感知页面及小型只读快照。复用避障核心的障碍网格与四路超声诊断，不重复订阅和处理原始深度。
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

页面操作见 [操作指南](../Doc/操作指南.md)，请求约定见 [Web 接口](../Doc/reference/Web接口.md)。本文中的模块路径相对 `server/`，涉及仓库其他目录时另行注明。

## 后端数据流

### 状态流

“避障可视化”不再提供独立观测开关。右上角系统菜单通过 `POST /api/avoidance/control` 提交布尔值 `enabled`：开启时启动观测服务，再调用底盘 `/base_control_router/set_avoidance` 并等待回执；关闭先确认底盘退出避障，再停止观测服务，释放相机、超声和计算资源。底盘拒绝关闭时不停止观测；开启失败时回收本次新启动的观测服务。启停请求串行处理。切换前需松开控制或取消导航。菜单状态来自底盘每 0.5 秒发布的 `/base_control_router/avoidance_enabled`，超过 2.5 秒未更新显示状态未知并禁用操作；不使用启动参数推断运行状态。

独立观测控制和重启接口已删除，观测启停统一使用避障开关。`finav-perception.service` 持有相机、超声和避障进程；切换页面不停止服务。`GET /api/perception` 的 `service` 字段复用后台每秒刷新的服务状态，提供启动、停止、失败状态及日志错误，不为每次网页轮询执行 systemctl。

感知页面通过共享 WebSocket 订阅 `perception`，仅页面可见时以最高 10 Hz 接收；`GET /api/perception` 仅保留为诊断快照。桥接订阅 `/avoidance/obstacles`（PointCloud，队列 1，5 Hz）和 `/avoidance/status`（DiagnosticArray，队列 1，5 Hz）。点数上限 4096，0.8 秒未更新不再显示深度点；诊断停更 2.5 秒后超声回波失效。浏览器断线或页面数据超过 1.5 秒未刷新时移除回波。点云版本和有效状态未改变时复用上次轮廓；距离读数不变时不重建 DOM。页面以 20 FPS 平滑超声距离，每帧逼近新读数的 65%，只对有效读数插值，不保留失效残影。

车体几何和传感器安装位置来自节点加载的 URDF。左右侧超声独立于深度显示；后两路未接入。深度投影按约 10 cm 分格，追踪暴露边界并圆滑转角，保留空隙和孔洞，不做凸包填充；每份快照仅生成一次轮廓。绘制顺序为深度轮廓、车体、超声回波。超声仅保留测距末端短弧，不显示传播弧线，弧宽不代表真实物体宽度。背景只保留比例尺。进入该页不停止建图或导航，返回原任务页也不会停止任务；离开至另一任务页仍执行原有确认逻辑。

`ros_bridge.py` 后台启动一个 ROS2 节点 `web_control_server`：

- 订阅 `/map`
  - 对栅格内容计算签名，仅在内容变化时更新 `scene.map` 和递增 `map_version`
  - 状态仓库存储原始 `int8` 字节，传输时 zlib 压缩；同一地图版本复用压缩结果
- 订阅 `/planning_map`
  - 更新规划约束覆盖层，使用独立的 `planning_map_version`
- 订阅 `/scan`
  - 仅在网页存在可见实时画面时处理
  - 距离量化为毫米级小端 `uint16` 二进制，浏览器侧完成极坐标投影；处理频率跟随当前最高场景订阅频率
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

实时状态统一通过 `/api/realtime` 订阅。`scene` 建图最高 10 Hz、导航最高 15 Hz；`perception` 最高 10 Hz；`status` 最高 2 Hz，页面默认 1 Hz，后台降至每 3 秒一次。各主题独立限频，共用一条浏览器连接。协议及二进制格式见 [Web 接口](../Doc/reference/Web接口.md#实时订阅)。

每条连接只允许一批数据等待确认；浏览器完成解码和场景绘制后才发送 ACK。慢客户端下一次只读取最新快照，10 秒未确认断开，不积压历史帧。未变化的地图、规划图、路径和雷达不重复发送；新连接总是全量同步，因此重启和重连不依赖旧会话版本。订阅集合改变时重建连接并全量同步。

前端通过 HTTP 调控制和管理接口：

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

Web 的“重启底盘控制”复用运行时 restart 接口，在停止底盘后调用
`base_control.pcan_reset` 复位唯一的 PCAN-USB，再启动底盘。设备占用、
权限不足或复位失败会保持底盘停止，并写入事件和 `base.log`；不会重启 Ubuntu。
依赖和权限要求见 [base_control README](../../base_control/README.md)。

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

传输脚本 `app-realtime.js` 在基础状态/API 脚本之后、页面脚本之前加载。页面共用 `appState`；地图编辑器通过 `window.finavMapEditor` 暴露入口。已删除 EventSource、状态/感知周期 HTTP 轮询和实时数据 Base64 路径。

Web UI 复用 `server/web/styles.css` 的颜色、圆角、间距及已有组件。选项下拉统一使用地图选择器的 `.nav-select` 样式；普通 `select` 通过 `app-select.js` 的 `enhanceSelects(root)` 增强，动态生成字段后也需调用。新增控件须检查选中、禁用、键盘操作，以及桌面和移动端菜单边界，避免重新引入浏览器原生弹出菜单。

画笔仅激活时绑定绘画事件，按需重绘；单次最多修改 100000 格，撤销最多 20 笔并限制历史栅格数量。栅格保存按源版本检查并原子替换，未修改像素保持原值。

状态中心展开期间暂停被遮挡的实时画面流，状态数据仍按低频更新。普通速度和档位设置不增加周期轮询。

## 开发启动

已安装服务的环境按 [部署与运维](../Doc/部署与运维.md) 管理服务。

Web 使用 `requirements/web.txt` 中的 `websockets` Sans-I/O 实现协议解析，不自行实现 WebSocket 帧协议。浏览器需支持 `DecompressionStream("deflate")`。当前无 RGB 视频预览；后续视频应复用相机采集并接入独立视频通道，不把原始 RGB-D 塞入遥测连接。

仅在未安装服务的开发环境中，可从工作区根目录手动启动 Web：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 src/finav/server/run_server.py --host 0.0.0.0 --port 8010
```

## 调试建议

- 看 server 是否起起来：`/api/health`
- 看 ROS bridge 是否正常：`/api/status`
- 看实时连接：浏览器 Network 的 `/api/realtime`，以及 `/api/status` 中 `transport.realtime_clients` 的各连接速率、压缩后带宽、ACK 往返耗时和待确认状态
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

历史离线回归记录见 [开发与验证](../Doc/开发与验证.md#web-离线验证)；测试脚本已按用户要求于 2026-10-30 清理，所列旧命令当前不可直接执行。
