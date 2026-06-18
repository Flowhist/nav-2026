# 语音-导航对接实现说明

本文说明如何把语音语义模块接入当前 `finav` 导航链路，实现“导航到 spot1”这类语音地点导航。

## 1. 总体链路

当前项目已经提供地点导航桥接节点 `nav_voice_bridge.py`，推荐语音侧只输出“地点名”，不要直接发布坐标或控制底盘。

```text
语音输入
  -> 语音识别/语义解析
  -> /nav_voice_bridge/voice_command
  -> nav_voice_bridge.py
  -> /goal_pose
  -> nav_path_plan.py
  -> /plan
  -> nav_control.py
  -> /nav_cmd_vel
  -> base_control_router.py
  -> /cmd_vel
  -> base_control.py
```

核心边界：

- 语义侧负责：把自然语言解析成标准地点名，例如 `spot1`。
- 导航侧负责：把地点名查成地图坐标，发布 `/goal_pose` 并执行导航。
- 控制仲裁侧负责：处理摇杆抢停、键盘/Web/导航优先级和底盘故障状态；语义侧需要感知这些事件时直接订阅 router 状态话题。

## 2. ROS 话题接口

### 语音输入到导航桥接

```text
topic: /nav_voice_bridge/voice_command
type:  std_msgs/msg/String
data:  地点名，例如 "spot1"
```

示例：

```bash
ros2 topic pub --once /nav_voice_bridge/voice_command std_msgs/msg/String "{data: 'spot1'}"
```

### 导航桥接状态反馈

```text
topic: /nav_voice_bridge/status
type:  std_msgs/msg/String
data:  状态文本
```

常见状态：

- `navigating to spot1`
- `unknown location: spot1`
- `no locations loaded`

该状态只表示地点指令是否被桥接节点接受并转换成 `/goal_pose`，不表示导航已经到达或失败。

### 控制仲裁状态反馈

```text
topic: /base_control_router/status
type:  std_msgs/msg/String
data:  控制仲裁事件
```

常见状态：

- `joystick_stop:nav`
- `joystick_reset`
- `joystick_reset_required:nav ignored`
- `base_fault_active`
- `base_fault_cleared`

语音上游如果要在摇杆抢停、底盘故障、未复位时暂停或拒绝新语音导航，直接订阅这个话题处理即可。

### 导航目标输出

```text
topic: /goal_pose
type:  geometry_msgs/msg/PoseStamped
frame: map
```

`nav_voice_bridge.py` 会根据地点表自动发布 `/goal_pose`，语音侧不需要直接发布这个话题。

## 3. 地点注册与地点文件

地点数据保存在当前地图目录下：

```text
maps/<map_name>/<map_name>.locations.yaml
```

文件格式：

```yaml
locations:
  spot1:
    x: 1.234
    y: 2.345
    yaw_deg: 90.0
```

字段含义：

- `x`: 地图坐标系下的目标 x 坐标，单位 m。
- `y`: 地图坐标系下的目标 y 坐标，单位 m。
- `yaw_deg`: 到点后的目标朝向，单位 deg。

注册地点推荐使用现有工具：

```bash
ros2 run finav annotate_tool.py
```

如果运行中修改了 `.locations.yaml`，需要让桥接节点重新加载：

```bash
ros2 service call /nav_voice_bridge/reload std_srvs/srv/Trigger {}
```

## 4. 启动方式

导航 launch 默认可以选择地图。启用语音导航桥接时加参数：

```bash
cd ~/nav_workspace
source install/setup.bash
export FINAV_REPO_DIR=$PWD/src/finav
export FINAV_MAPS_DIR=$PWD/src/finav/maps

ros2 launch finav nav.launch.py use_nav_bridge:=true
```

如果不启用 `use_nav_bridge:=true`，语音侧发布 `/nav_voice_bridge/voice_command` 不会触发导航。

## 5. 语义侧适配

语义侧只需要把语音命令归一化为地点名。

输入示例：

```text
导航到 spot1
去一号点
带我去充电区
```

语义输出建议：

```json
{
  "intent": "navigate_to_location",
  "location": "spot1"
}
```

ROS 发布：

```text
/nav_voice_bridge/voice_command = "spot1"
```

建议语义侧完成这些归一化：

- 去掉命令词，例如“导航到”“去”“带我去”。
- 把中文别名映射到标准地点名，例如“一号点” -> `spot1`。
- 对无法识别的地点，不发布导航命令，直接向用户反馈“未找到地点”。

## 6. 推荐最小实现

语义模块内部维护一个别名字典：

```yaml
aliases:
  spot1: ["spot1", "一号点", "第一个点"]
  charge: ["充电区", "充电点", "charge"]
```

处理流程：

```text
1. 语音识别得到文本
2. 判断是否为导航意图
3. 提取地点短语
4. 通过别名字典映射到标准地点名
5. 发布 std_msgs/String 到 /nav_voice_bridge/voice_command
6. 订阅 /nav_voice_bridge/status 获取执行反馈
```

## 7. 快速测试

启动导航：

```bash
ros2 launch finav nav.launch.py use_nav_bridge:=true
```

查看地点桥接状态：

```bash
ros2 topic echo /nav_voice_bridge/status
```

模拟语音命令：

```bash
ros2 topic pub --once /nav_voice_bridge/voice_command std_msgs/msg/String "{data: '电梯口'}"
```

确认目标点已发布：

```bash
ros2 topic echo /goal_pose --once
```

确认路径规划输出：

```bash
ros2 topic echo /plan --once
```

确认导航速度输出：

```bash
ros2 topic echo /nav_cmd_vel
```

## 8. 后续扩展

如果希望导航侧直接支持别名，可以扩展地点文件：

```yaml
locations:
  spot1:
    x: 1.234
    y: 2.345
    yaw_deg: 90.0
    aliases: ["一号点", "第一个点", "spot one"]
```

然后在 `nav_voice_bridge.py` 中增加 alias 查找逻辑。当前最小实现建议先把别名处理放在语义侧，导航侧只接收标准地点名，这样边界最清晰。
