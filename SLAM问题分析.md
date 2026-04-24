# SLAM 问题分析

## 现象

- 实机建图和定位时容易出现运动尺度偏小的问题，尤其是直行阶段更明显。
- 典型表现是：实机前进约 10m，RViz / SLAM 中只累计出约 9m。
- 当前链路为：`/odom_encoder` -> EKF -> `/odom` -> `slam_toolbox`，激光输入为 `/scan`。

## 当前实现里的高风险点

### 1. 底盘线速度尺度最可疑，里程计的距离换算完全依赖 `wheel_radius`

- `config/base_control.yaml:6-10` 中，底盘里程计只靠 `wheel_radius=0.145`、`wheel_separation=0.50` 和 `10Hz` 更新。
- `scripts/control/base_control.py:248-256` 中，`get_velocity([1, 2])` 返回的轮速度会被直接乘上 `wheel_radius` 换算成线速度。
- `scripts/control/base_control.py:258-265` 中，前向距离完全由 `v_forward * dt` 积分得到。

这意味着只要下面任意一项偏小，系统里程就会整体缩短：

- `wheel_radius` 配小了；
- `get_velocity()` 的单位理解错了；
- 驱动返回的是“电机侧速度”而不是“轮边等效速度”；
- 低频积分漏掉了加减速段的一部分速度面积。

如果现象稳定接近“10m 只记 9m”，这更像尺度系数问题，而不是随机噪声。

### 2. EKF 只吃编码器的 `vx`，所以线距离误差会直接透传到 `/odom`

- `config/ekf.yaml:24-34` 中，`/odom_encoder` 只融合了 `vx` 和 `vyaw`，不融合里程计位置。
- `launch/sub/ekf.launch.py:17-27` 中，EKF 输出 `/odom`，这正是后面 `slam_toolbox` 用的 odom 先验。

这带来的结果是：

- 如果底盘里程计的 `vx` 本身偏小，EKF 输出的 `/odom` 线位移也会偏小；
- 因为 EKF 没有别的线速度观测源，系统里不存在任何一层去主动修正这个尺度。

### 3. `slam_toolbox` 不一定能把“前进距离偏小”自动拉回来

- `config/slam_toolbox_map.yaml:43-48` 中，建图模式只有走到 `0.2m` 或姿态变化到阈值才强制触发更新。
- `config/slam_toolbox_nav.yaml:39-50` 中，定位模式更保守，`minimum_time_interval=0.2` 且 `throttle_scans=2`。
- 你当前雷达前装、前轮遮挡，前向有效视场大约只有 160 度，直行场景里可用于约束前进尺度的特征并不强。

所以这里更合理的判断是：`slam_toolbox` 主要在用 `/odom` 提供初值，而不是每一段直线都强行把里程尺度重新校正。当前场景下，底盘里程如果先天就少算 10%，scan matching 很可能只会跟着这个先验跑，而不是每次都把它拉准。

### 4. IMU 不是这次“距离偏小”的主因，但仍有配置风险

- `config/ekf.yaml:38-47` 中，EKF 对 IMU 只融合了姿态里的 `yaw`，没有融合角速度 `vyaw`。
- `scripts/imu/dm_imu_publisher.py:181-199` 中，代码已经明确写了“6轴IMU的Yaw会漂移”。

这条更偏向“航向会不会慢慢歪”，不是这次“10m 变 9m”的第一嫌疑。但它依然会削弱 scan matching 的纠偏效果，所以仍然值得后续处理。

### 5. IMU 和雷达的安装外参目前都按“理想安装”处理

- `urdf/whillcar.urdf:157-177` 中，`imu_link` 和 `laser_frame` 的 `rpy` 都是 `0 0 0`。
- `config/imu.yaml:17-19` 中，`yaw_offset_deg` 现在也是 `0.0`。
- `urdf/whillcar.urdf:39-41` 中，雷达被建模在 `x=0.62, y=0.26, z=0.42`，也就是偏前、偏左的位置。

如果 IMU 实际安装有一点点偏角，或者雷达真实安装位姿和 URDF 不一致，SLAM 会在 TF 上吃到系统性误差。它不一定直接造成“距离少算 10%”，但会让 scan matching 更难纠正底层 odom 的尺度误差。

## 次级风险

### 6. SLAM Toolbox 参数还偏向旧单雷达工况，部分阈值对纠正里程尺度不友好

- `config/slam_toolbox_map.yaml:43-60` 中，建图模式的 `minimum_time_interval=0.1`、`minimum_travel_distance=0.2`、`minimum_travel_heading=0.1`，更新不算积极。
- `config/slam_toolbox_nav.yaml:39-57` 中，定位模式更保守：`scan_queue_size=3`、`minimum_time_interval=0.2`、`throttle_scans=2`、`minimum_travel_heading=0.15`。
- 两份配置里的 `minimum_angle/maxium_angle` 仍然是 `[-pi, pi]`，但当前雷达真实输出已经是 `-45° ~ 225°`。

这些参数不会单独制造“距离缩短”，但会让 scan matching 的纠偏介入得更慢。特别是定位模式，本来就更依赖 odom 初值，当前配置下如果 `/odom` 的线位移先天偏小，SLAM 可能就会跟着这个偏小尺度跑。

### 7. 雷达消息时间戳来自设备时间，不是 ROS 本机时间

- `FREE_CH_02.3.01.001_ROS2/FREE_CH_02.3.01.001/src/rosnode/free_lidar_node.cpp:265-275` 中，`LaserScan.header.stamp` 使用的是 `scandata.sec/nsec`。

如果这套设备时间和主机 ROS 时间不同步，TF 查询和扫描匹配会引入额外时序误差。这个问题通常会先表现为偶发 TF 超时、抖动或匹配不稳定，不一定是当前直行距离偏小的主因，但值得列为排查项。

## 为什么直行时更容易少算距离

- 直行时系统几乎完全依赖 `vx` 积分来累计前进距离，轮速尺度一旦偏小，就会连续少算。
- 你当前的激光安装在车体前部且偏左，前轮还有遮挡，直线段中可用于约束“我到底走了多远”的几何特征不够强。
- 当前 SLAM 参数又更偏向“减负”和“稳态”，因此系统更容易先相信 `/odom`，而不是频繁用 scan matching 重估前进量。

## 建议的排查顺序

### 第一优先级

1. 先完全绕开 SLAM，只看 `/odom_encoder`：让车实机直走固定距离，例如 5m 或 10m，比较 `/odom_encoder` 累积的 `x` 是否同样偏小。
2. 如果 `/odom_encoder` 本身就少算，问题就在底盘速度尺度，不在 SLAM。
3. 优先回查 `wheel_radius` 和 `get_velocity()` 的真实单位。

如果 `/odom_encoder` 已经是 10m 只记 9m，那么主因基本可以锁定为里程计尺度。

### 第二优先级

1. 标定底盘里程计参数，先盯 `wheel_radius`，再看 `wheel_separation`。
2. 记录长直线下左右轮返回速度，确认 `get_velocity()` 返回值是不是和代码假设的 `deg/s` 一致。
3. 核对一次“实机轮胎有效滚动半径”和 `0.145m` 是否真的相符。

### 第三优先级

1. 在保持底盘匀速直行的情况下，同时看 `/odom_encoder` 和 `/odom`，确认 EKF 有没有进一步把距离压小。
2. 适当调大 `/odom_encoder` 协方差，避免 EKF 过度确信这份理想化速度。
3. 如有需要，再把 IMU 从“绝对 yaw 融合”改成“角速度辅助”，防止它间接干扰 scan matching。

### 第四优先级

1. 复核 IMU 实际朝向和 `imu_link` 是否一致。
2. 复核雷达实际安装位姿和 `laser_frame` 是否一致。
3. 如果 IMU 安装不可能完全摆正，至少补上 `yaw_offset_deg` 或 URDF 里的固定旋转。

### 第五优先级

1. 重新调 `slam_toolbox_map.yaml` 和 `slam_toolbox_nav.yaml`，降低 `minimum_time_interval`、`minimum_travel_heading`，减少定位模式下的 `throttle_scans`。
2. 把激光角度参数改成与当前雷达真实扇区一致，避免继续沿用全向雷达时期的假设。

### 第六优先级

1. 检查运行日志里是否有 TF extrapolation 或 timestamp 相关警告。
2. 如有必要，临时把 `LaserScan.header.stamp` 改为 ROS 当前时间，验证时间戳是否参与了距离估计问题。

## 结论

当前最可疑的根因有两个：

1. 编码器线速度尺度偏小，最可能落在 `wheel_radius` 或 `get_velocity()` 单位理解上。
2. `slam_toolbox` 在当前前向视场受限的工况下，没有足够强的几何约束去主动把这份“少算距离”的 odom 拉正。

你现在考虑上双雷达，对 scan matching 的可观测性会有帮助，但它主要解决的是“激光几何约束不够强”的问题。若不先把底盘里程计的线速度尺度校准好，双雷达只能缓解，不能根治。
