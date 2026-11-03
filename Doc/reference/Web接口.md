# Web 接口

本文集中维护 HTTP 与 WebSocket 接口约定。页面操作见 [操作指南](../操作指南.md)，内部数据流见 [Web 开发说明](../../server/README.md)，部署和访问控制见 [部署与运维](../部署与运维.md#web-访问)。

## 基础状态

- `GET /api/health`
- `GET /api/status`
- `GET /api/perception`（只读诊断快照，页面使用 WebSocket）
- `GET /api/history?since=<seq>&limit=<n>&session_id=<id>`（响应含 `session_id`、`latest_seq`、`reset`、`truncated`）

## 实时订阅

连接同源 `/api/realtime`，子协议 `finav.realtime.v1`。与 HTTP 共用身份验证；拒绝跨源 Origin。支持协商标准 `permessage-deflate`，浏览器自动压缩/解压消息；不支持该扩展的客户端也可连接。旧 `/api/stream`、`/api/scene` 已删除，旧客户端需要迁移。

连接后 5 秒内发送一次订阅，例如：

```json
{"type":"subscribe","mode":"navigation","topics":{"scene":15,"status":1}}
```

主题为 `scene`、`perception`、`status`，请求频率必须为有限正数，服务端分别限至 15、10、2 Hz；`mode=mapping` 的场景限至 10 Hz。各主题独立计时，未订阅的数据不传输。变更订阅集合须重新连接；每个新连接都从完整快照开始，不接受上次连接的地图版本。

服务端发送一条二进制 WebSocket 消息：

```text
FNV1 (4 bytes) | JSON length (uint32 little-endian) | UTF-8 JSON | binary attachments
```

JSON 包含 `protocol=1`、`session_id`、连续递增 `seq`、`server_sent_at`、本批次到期主题以及 `attachments` 描述数组。附件字段为 `path`、`offset`、`length`、`decoded_length`、`encoding`、`compression`；偏移相对二进制附件区起点，长度均为字节数。

| 路径 | 编码 | 压缩 |
| --- | --- | --- |
| `scene.map.data` | `int8`，保留 -1 未知栅格 | `deflate`，带 zlib 头 |
| `scene.scan.ranges` | `uint16-le`，毫米，0 表示无效距离 | `none` |
| `perception.points` | `float32-xy-le`，连续 x/y，单位米 | `none` |

规划覆盖层继续使用紧凑 `blocked-ranges-v1`；小型位姿、控制状态、路径和诊断保留 JSON。地图/规划图/路径只在版本变化时发送；未变化时相应 `*_changed=false` 且值为 null，不能清除客户端缓存。`*_changed=true` 且值为 null 才表示清空。未变化的雷达字段省略。雷达 `stamp` 保留 ROS 采集时间并配对采集时刻 TF；`server_sent_at` 仅为服务端墙上时钟，未经时钟同步不能据此声称精确单程延迟。

客户端解码、消费完本批数据后发送：

```json
{"type":"ack","seq":1}
```

每连接只允许一批未确认数据。未确认期间不排队、不构建后续帧；确认后重新读取最新状态。10 秒未确认关闭连接。WebSocket Ping/Pong 由协议库处理；活跃订阅的数据/ACK 同时用于检测连接健康。客户端消息上限 8 KiB，单批及单附件解压后上限 32 MiB；最多 8 条实时连接。非法消息关闭连接且不会进入机器人命令队列。

`GET /api/status` 的 `transport.realtime_clients` 暴露逐连接主题、发送字节/秒、实际批次频率、ACK 往返毫秒数和待确认状态；`realtime_bytes_per_sec` 为各连接数据帧带宽之和，按协商压缩后的实际发送字节统计，含 WebSocket 数据帧头、不含 TCP 及 Ping/Pong。HTTP 控制和地图管理请求不通过该通道。

## 地图

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

## 导航 / 控制

- `POST /api/nav/goal` `{ x, y, yaw_deg }`
- `POST /api/nav/initialpose` `{ x, y, yaw_deg }`
- `POST /api/nav/cancel`
- `POST /api/nav/route` `{ action, map_name, route_id }`，`action` 为 `start/pause/resume/cancel`
- `POST /api/nav/location` `{ name }`（命中当前地图路线的 `end_name` 时巡线，命中零散地点的 `name` 时正常导航）
- `POST /api/nav/relocate` `{ map_file }`
- `POST /api/control/manual` `{ linear, angular }`（归一化输入，非实际速度；零输入表示停止）

ROS 控制与存图接口返回后仍需查看状态和事件确认执行结果；存图通过后台脚本完成。

口令登录默认关闭，只有显式设置 `FINAV_WEB_AUTH=1` 才启用 Basic 身份验证。写请求使用 `application/json`，拒绝跨站来源；无效参数返回 4xx，ROS 离线或普通命令队列已满返回 503。手动输入保留最新值并在 300 毫秒后过期，停止/取消指令优先处理。

## 运行时

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

## 配置文件

- `GET /api/configs`
- `GET /api/configs/<filename>`
- `POST /api/configs/<filename>` `{ content, expected_revision }`（读取时取得 `revision`；冲突返回 409，YAML/基本结构校验失败返回 422）

## Finav 服务

- `GET /api/system`
- `POST /api/system/shutdown`
- `POST /api/system/restart`

## 继续建图

`POST /api/runtime/mapping/start` 接受 `{ "map_file": "旧图名" }`；空对象保持新建图行为。
定位使用 `POST /api/mapping/initialpose`（`x/y/yaw_deg`）或 `POST /api/mapping/relocate`；开始续建使用 `POST /api/mapping/resume`（空对象即可）。存图和停止复用现有接口。

## 速度与档位

- `GET /api/nav/speed`、`POST /api/nav/speed`：读取、保存导航速度配置。
- `POST /api/control/gear`：设置共用手柄档位。

导航参数为 `linear_speed_mps`、`angular_speed_degps`、`drive_angular_speed_degps`，位于 `config/nav.yaml` 的 `nav_control.ros__parameters`。保存保留其他字段与注释，运行中通过 `/nav_control/set_speed` 一起应用；离线或应用失败会明确反馈，保存成功不等于当前节点已应用。

ROS 设置服务使用 `rcl_interfaces/srv/SetParametersAtomically`：导航接受以上三个参数，手柄 `/handle_control/set_gear` 接受 `gear` 整数。手柄写十进制寄存器 23，再读取寄存器 3 确认，通过 `/handle/gear` 同步状态；修改节点后需重启加载接口。
