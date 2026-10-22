# Gemini 305 人员跟随 demo

独立进程：Orbbec 官方 Python SDK 直接取 RGB-D，YOLOv8n + ByteTrack
识别人并锁定 ID，按方位和深度输出线速度、角速度。无 Nav2、地图、路径管理、
图像 ROS topic 或常驻服务，不加入默认 launch。

## 环境

Python 3.10；本机独立环境 `/home/embotic/nav_workspace/.venv-person-follow`。
依赖不写入系统 Python。普通 ARM64 PyTorch wheel 作为 CPU 功能基线，
不代表 Jetson CUDA 已启用。后续可替换匹配 JetPack 的 PyTorch 或使用导出的 TensorRT 模型。

```bash
uv venv --python /usr/bin/python3 --system-site-packages /home/embotic/nav_workspace/.venv-person-follow
uv pip install --python /home/embotic/nav_workspace/.venv-person-follow/bin/python -r /home/embotic/nav_workspace/src/finav/demo/person_follow/requirements.txt
uv pip install --python /home/embotic/nav_workspace/.venv-person-follow/bin/python --no-deps pyorbbecsdk2==2.1.2
```

官方 API 参考：[Orbbec 对齐示例](https://github.com/orbbec/pyorbbecsdk/blob/v2-main/examples/beginner/03_color_and_depth_aligned.py)、
[Ultralytics tracking](https://docs.ultralytics.com/modes/track/)。模型使用 Ultralytics 的许可条款。
第一次运行 YOLO 会下载权重到本目录；SDK 默认日志取决于运行目录。

## 默认观察模式

```bash
cd /home/embotic/nav_workspace/src/finav/demo/person_follow
/home/embotic/nav_workspace/.venv-person-follow/bin/python run.py --seconds 120 --rotate180
```

仅使用终端，无窗口、截图或图形桌面依赖；`--show` 和 `--snapshot` 已移除。
终端原地刷新目标表；重定向到文件时逐帧追加文本。按 Ctrl+C 退出。
本机首次实测画面倒置，因此示例启用 `--rotate180`，同时旋转彩色与对齐深度；
相机恢复正向安装后去掉此参数。三维投影同步调整水平和垂直主点，不能只翻转显示。
默认不初始化 ROS、不发布任何速度。目标表显示模型检测到的各类物体，包括类别、
置信度、ID、车体坐标 XYZ、平面距离和方位角；仅 person 参与跟随目标选择。
`*` 标记当前锁定的人，深度不足显示 `depth invalid`，不编造坐标。
非人体也沿用框内局部深度估计，仅用于观察，不表示完整物体的几何中心。
末尾显示 STOP/FORWARD/LEFT/RIGHT、建议速度、帧延迟和停车原因。
控制模式另显示四路前向超声测距、最近一次 `/follow_cmd_vel` 输出及超声/超时/取消门控状态，
该输出仍是仲裁器输入，不是实际底盘反馈。无新图像时刷新等待状态，不保留旧位置当作实时位置。

## 相机安装位姿

2026-10-16 暂用测量值统一保存在 `urdf/whillcar.urdf` 的四个
`front_camera_*` 数值属性中；demo 启动时直接读取，不需要运行 TF 或 xacro。
`--robot-model` 可指定另一份模型。修改这些属性后重启 demo 生效。

| 参数 | 暂用值 |
| --- | --- |
| x：向车前 | 0.575 m |
| y：向车左 | 0.2775 m |
| z：距地面 | 0.727 m |
| yaw：相对车头，左转为正 | -30 度（偏右） |

`base_link` 是后驱动轮轴心中点的地面投影；不是轴心高度平面。
暂把测量点当作彩色相机光心，俯仰为零；机壳到光心偏移、镜头畸变未进一步标定。
URDF 中 `front_camera_link` 采用前/左/上轴，
`front_camera_upright_optical_frame` 采用右/下/前轴，对应纠正倒置后的图像，
不能直接用于未翻转的 SDK 原始图像。

SDK 深度按光轴 Z 使用，结合内参还原目标三维位置，再按相机 yaw 旋转并加安装平移。
表格 XYZ 是车体坐标，`DIST=hypot(x,y)`，`BEARING=atan2(y,x)` 以度显示。
高度用于目标 z 估计，不计入平面跟随距离。

初次画面仅有一人时，连续三次追踪结果确认后锁定。多人时等待，不随意选人。
目标丢失立即请求零速，不自动切换 ID；需要换人时退出后重启。
`--distance` 现在从 `base_link` 计量，默认 2.1 m，允许 1.8～4 m。
默认值由原来的相机前方 1.5 m 调整而来，避免改换距离参考点后缩短跟随间距；
它不是车头净空距离。按 URDF 车头 x=0.84 m，正前目标距原点 2.1 m 时，车头净空约 1.26 m。
不倒车；超过 25 度先转向；上限 0.15 m/s、0.3 rad/s。

## 控制模式（现场确认安全后）

先停止现有路径跟踪节点，确保只有本 demo 向 `/follow_cmd_vel` 发布。
底盘仲裁器和 DCCS 的独立 dauxi 超声驱动必须已运行；本 demo 不启动底盘或传感器服务。
跟随不订阅 `/scan`，不需要启动雷达、建图、导航或 Web。
相机必须固定、保持水平俯仰，平面安装偏角按上述外参补偿；先用观察模式核实方向和测距。

### 四路超声配置与安装位置

跟随仅使用前左、前右、左侧、右侧四路（0x10～0x13），不使用后向两路。
2026-10-16 逐位置遮挡测试确认了这四路地址。以下坐标由用户提供，
从厘米换算为米；俯视原点为后驱动轮轴心中点，+x 向前、+y 向左。

| 位置 | 地址 | x (m) | y (m) | 地址验证状态 |
| --- | --- | --- | --- | --- |
| 前左 | 0x10 | 0.64 | 0.31 | 已实测 |
| 前右 | 0x11 | 0.64 | -0.31 | 已实测 |
| 左侧 | 0x12 | 0.57 | 0.32 | 已实测近远变化 |
| 右侧 | 0x13 | 0.57 | -0.32 | 已实测 |
| 后左 | 待确认（候选 0x14） | -0.10 | 0.1675 | 最后一轮未执行 |
| 后右 | 暂定 0x15 | -0.10 | -0.1675 | 近距离回波，待后左交叉确认 |

当前停车判断使用各探头的原始距离，不使用坐标做障碍物投影。
坐标仅作安装记录；高度、准确朝向和声束角尚未提供，不生成假定的 TF 或二维障碍物点。
独立 `ultrasonic.yaml` 仅轮询四路，沿用 DCCS 驱动和设备路径，
探头间隔 30 ms；不修改 DCCS 原有六路配置。
先确认串口未被其他进程占用，再单独启动现有驱动（不启动 DCCS 全栈）：

```bash
source /opt/ros/humble/setup.bash
source /home/embotic/DCCS/install/local_setup.bash
ros2 run dauxi dauxi --ros-args -r __ns:=/ultrasonic \
  -p config_file:=/home/embotic/nav_workspace/src/finav/demo/person_follow/ultrasonic.yaml -p enable:=1
```

另一个终端仅监听测距，不连接底盘、不发布速度：

```bash
source /opt/ros/humble/setup.bash
source /home/embotic/DCCS/install/local_setup.bash
cd /home/embotic/nav_workspace/src/finav/demo/person_follow
python3 ultrasonic.py --seconds 120
```

终端显示位置名称、地址、距离、样本年龄，以及同一停车门控的 STOP/READY 状态；
READY 仅表示超声门控通过，不代表人员跟随已准备好。工具不发布任何速度。
可用 `--stop-distance` 调整验证阈值（默认 0.5 m），控制程序对应参数为 `--ultrasonic-stop`。
在四个位置分别放置/移走障碍物，应出现 STOP/READY；停止驱动后应出现过期 STOP。
2026-10-16 前左障碍物测试中，0x10 连续五次为 0.065 m，20 次四路读取均通过 CRC。
此结果只验证测距，未验证实际底盘制动。0x11 无近障时常为 5.657 m，
其无回波/量程含义尚未查明；程序未将该数值定义为故障码，不能据此证明前方安全。
数值不变但时间戳持续更新也不能证明探头正常。

### 启用跟随

已默认配置这四路，无需再手动填写地址。保留的 `--front-addresses` 参数只接受这四个地址，
不能把后向探头替换进跟随门控。

```bash
source /opt/ros/humble/setup.bash
source /home/embotic/DCCS/install/local_setup.bash
source /home/embotic/nav_workspace/install/local_setup.bash
cd /home/embotic/nav_workspace/src/finav/demo/person_follow
/home/embotic/nav_workspace/.venv-person-follow/bin/python run.py --seconds 30 --rotate180 --drive
```

默认订阅 `/ultrasonic/device/state`（`interfaces/msg/KS114State`），总线 `bus_0`；
可用 `--ultrasonic-topic`、`--ultrasonic-bus` 指定已有驱动的话题和总线。
`--ultrasonic-stop` 默认 0.5 m，指从探头起算的距离，不是车体原点或相机距离。
人员跟随间距仍由 RGB-D 的 `--distance` 决定；超声用于近障停车，不用于识别人。

控制经 `/follow_cmd_vel -> base_control_router -> /cmd_vel`，不绕过仲裁。
跟随取消使用独立 `/follow_clear`；导航继续使用 `/nav_cmd_vel` 和 `/nav_clear`。
仲裁器同时收到有效导航与跟随消息（包含零速）时取消两者并保持自动控制停车，
直到两个输入都超时才解除冲突。必须退出已取消的 demo，避免持续零速阻止导航恢复。
键盘/Web 的原有优先级保留，手柄可抢停跟随。底盘须先构建并在现场安全时重启，
旧版本仲裁器不订阅 `/follow_cmd_vel`，只改 demo 不会驱动车辆。
独立 20 Hz 输出循环：帧结果超过 350 ms，或超声消息/任一路样本超过 500 ms，
或任一路缺失、重复、离线、测距无效，或任一路距离不大于 0.5 m，均输出零速（含角速度）。
逐路检查采样时间戳，驱动重发缓存不能续期；不依赖汇总消息的发布时间。
出现第二个跟随速度发布者或收到 `/follow_clear` 后锁定取消，必须重启 demo 才能恢复。
目标框中心躯干深度稀疏或离散时停车；RGB-D 不匹配、相机异常退出时清理并停车。

这只是有人看护的低速验证：前向及侧向四路超声不能覆盖后方或完整转弯扫掠范围，
存在盲区及软物体、斜面回波问题，未实现绕障、地面落差检测或长期身份重识别。
相机外参目前使用粗测值，仅做平移与偏航补偿。
SDK/推理观测通过不等于运动验收通过。CPU 处理超过 350 ms 时控制会保持零速，
应先优化推理，不能靠放宽超时掩盖延迟。

## 离线验证

```bash
python3 -m unittest discover -s /home/embotic/nav_workspace/src/finav/demo/person_follow -p 'test_*.py'
```

2026-10-16 本机验证：官方 SDK 2.1.2、YOLOv8n CPU 完成 10 秒 RGB-D 观察，
处理 33 帧，延迟多数 230～340 ms，少数超过 350 ms。首次倒置视野未锁定目标。
启用 180 度翻转后的第二次 5 秒观察处理 15 帧，识别左侧真人并连续锁定 ID 1，
有效深度约 0.31～0.49 m，无效深度帧与距离过近时请求速度均为零。
这些数值未用外部量具核准。尚未完成前方人员行走、遮挡和多人交叉的现场验证，
也未执行任何底盘运动。
核心测试覆盖方向/停车、无效深度、锁定后不切人、推理过期与发布冲突取消。
安装位姿补偿新增测试覆盖相机中心射线、车体正前目标及倒置图像主点修正。
外参更新仅完成离线验证，未重新取流或执行运动。
