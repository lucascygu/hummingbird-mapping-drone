# PX4 参数：FAST-LIO2 外部视觉（EV）融合 + TELEM2 uXRCE-DDS

本文档由 Grok 撰写/协助；2026-11-11 由 Claude 按 PX4 v1.16.1 重新核对并改写。

> 适用：Pixhawk 6C + PX4 **v1.16.1**（团队 2026 年 3 月报告记录的固件版本）。每个参数名、默认值和行为都对照 PX4 官方仓库 tag `v1.16.1` 的源码核对过，出处写在表里。
> **没有在 SITL 或真机上验证过。** 刷固件后请在 QGC 里逐个确认参数存在。
> 本文件只涉及估计器输入和通信，**不包含任何解锁、起飞或 offboard 设置**。

## 0. 原文是按 v1.15 写的，1.16.1 有这些不同

| 项 | v1.15（原文） | v1.16.1（实际固件） |
| --- | --- | --- |
| `EKF2_EV_CTRL` 默认值 | 15 | **0**。不手动设置就完全不融合外部视觉 |
| EV 参数定义所在文件 | `src/modules/ekf2/module.yaml` | `src/modules/ekf2/params_external_vision.yaml` |
| `/fmu/in/vehicle_visual_odometry` 在 `dds_topics.yaml` 的行号 | 136 | 142。话题名不变：`VehicleOdometry` 的 `MESSAGE_VERSION` 是 0，不加后缀 |
| 带版本号的话题 | 没有 | `MESSAGE_VERSION` 不为 0 的消息带 `_v<N>` 后缀，例如飞控状态是 **`/fmu/out/vehicle_status_v1`** |
| `px4_msgs` 分支 | `release/1.15` | **`release/1.16`**（`hummingbird.repos` 已改）。`VehicleOdometry` 字段相同，但另有 71 个消息定义不同 |
| uXRCE-DDS 新参数 | — | `UXRCE_DDS_PTCFG`、`UXRCE_DDS_SYNCC`、`UXRCE_DDS_TX_TO`、`UXRCE_DDS_RX_TO` |

## 1. TELEM2 ↔ Jetson（uXRCE-DDS 串口）

| 参数 | 建议值 | 默认值（1.16.1） | 说明 / 出处 |
| --- | --- | --- | --- |
| `MAV_1_CONFIG` | `0`（Disabled） | 实例 1 默认为空（禁用） | 确保 TELEM2 上没有 MAVLink 抢占串口。`src/modules/mavlink/module.yaml`：`default: [TEL1, "", ""]` |
| `UXRCE_DDS_CFG` | `102`（TELEM2） | 0 | 端口编号来自 `Tools/serial/generate_config.py`：`"TEL2": index 102`；参数定义在 `src/modules/uxrce_dds_client/module.yaml` |
| `SER_TEL2_BAUD` | `921600` | 921600 | 同上文件，TEL2 `default_baudrate: 921600` |
| `UXRCE_DDS_SYNCT` | `1` | 1 | 时间同步：client 把收到的 `timestamp` / `timestamp_sample` 减去 agent↔飞控时钟差（`Tools/msg/templates/ucdr/msg.h.em`）。**所以 Jetson 上发给 PX4 的时间戳必须是系统时钟，不能是仿真时钟** |
| `UXRCE_DDS_DOM_ID` | `0` | 0 | 与 Jetson 上 `ROS_DOMAIN_ID` 一致 |
| `UXRCE_DDS_KEY` | `1` | 1 | 单飞控单 agent 保持默认 |
| `UXRCE_DDS_PTCFG` | `0` | 0 | 0 默认；1 仅本机（配合 `ROS_LOCALHOST_ONLY=1`）；2 自定义 participant |
| `UXRCE_DDS_SYNCC` | `0` | 0 | 用 agent 的时间去设飞控系统时钟。保持关闭 |
| `UXRCE_DDS_TX_TO` / `UXRCE_DDS_RX_TO` | 默认 | 3 / −1 | 多少秒没有收发数据就重建 DDS 连接；小于 1 表示关闭 |

以上参数全部**需重启**飞控生效。板级串口映射：Pixhawk 6C 的 TELEM2 = `/dev/ttyS3`（`boards/px4/fmu-v6c/default.px4board`：`CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS3"`），默认固件已包含 `CONFIG_MODULES_UXRCE_DDS_CLIENT=y` 和 `CONFIG_MODULES_EKF2=y`。

团队报告里记录的 Pixhawk↔Jetson 链路是 MAVLink over UART，且最终接线图里这条线**还没有接**。本仓库用的是 uXRCE-DDS；TELEM2 只能二选一。

Jetson 端（agent 必须是 **v2.x**，PX4 文档说明 v3.x agent 与 PX4 client 不兼容）：

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600   # 设备名取决于载板，未在实机核对
# 或： ros2 launch hummingbird_bringup mapping.launch.py start_xrce_agent:=true xrce_dev:=/dev/ttyTHS1
```

接线：TELEM2 TX→Jetson RX、RX→TX、GND→GND（3.3 V 电平）。**不要接 TELEM2 的 5 V 到 Jetson。**

验证链路通了：

```bash
ros2 topic list | grep /fmu/                       # 应看到 /fmu/in/vehicle_visual_odometry 和一批 /fmu/out/*
ros2 topic echo /fmu/out/vehicle_status_v1 --once  # 注意 _v1 后缀
ros2 topic echo /hummingbird/px4_bridge/status     # px4_subscribers 应为 1，state 应为 ok
```

飞控一侧（QGC → Analyze Tools → MAVLink Console）：`uxrce_dds_client status`。

## 2. EKF2 外部视觉融合

| 参数 | 建议值 | 默认值（1.16.1） | 说明 |
| --- | --- | --- | --- |
| `EKF2_EV_CTRL` | 室内：`11`（bit0 水平位置 + bit1 垂直位置 + bit3 偏航）；室外有 GNSS：`3`（只开位置） | **0** | bit 定义：0 水平位置 / 1 垂直位置 / 2 三维速度 / 3 偏航。桥接节点默认**不发速度**（FAST_LIO_ROS2 不填 twist），所以不开 bit2；若开启 `estimate_velocity:=true` 再加 4 |
| `EKF2_HGT_REF` | 室内无 GNSS：`3`（Vision）；室外有 GNSS：保留 `1` | 1 | 枚举：0 气压 / 1 GPS / 2 测距 / 3 视觉。**需重启**。设为 3 的前提见下面"高度基准" |
| `EKF2_EV_DELAY` | 先 `0`（Mid-360 已 PTP 同步、桥接用 header 时间）；若桥接退回接收时间（日志会告警），从 `50` 起调 | 0 | 单位 ms，0–300，**需重启**。EKF 用 `timestamp_sample − EV_DELAY` 作为测量时刻 |
| `EKF2_EV_NOISE_MD` | `0` | 0 | 0 = 用消息里的方差、参数作下限；1 = 直接用参数 |
| `EKF2_EVP_NOISE` | `0.1` m | 0.1 | 位置噪声下限（最小 0.01） |
| `EKF2_EVA_NOISE` | `0.1` rad | 0.1 | 角度噪声下限（最小 0.05） |
| `EKF2_EVP_GATE` | `5` | 5 | 位置新息门限（标准差倍数） |
| `EKF2_EV_QMIN` | `0` | 0 | 桥接节点发 `quality=0`（未知），QMIN=0 时不检查 |
| `EKF2_EV_POS_X/Y/Z` | **`0`** | 0 | 桥接节点已用 `imu_to_base_xyz` 把位姿换算到 base_link（机体重心）。**如果这里再填杠杆臂，会重复补偿** |
| `EKF2_BARO_CTRL` | `1`（保留，气压作辅助） | 1 | |
| `EKF2_GPS_CTRL` | 室内：`0`；室外：默认 `7` | 7 | bit0 经纬度 / bit1 高度 / bit2 三维速度 / bit3 双天线航向 |
| `EKF2_MAG_TYPE` | 室内（钢结构干扰大）可设 `5`（None），由 EV 偏航提供航向 | 0 | 枚举：0 自动 / 1 磁航向 / 5 不用 / 6 仅初始化。**需重启**。室外有 GNSS 时保留默认 |

`EKF2_EV_*` 来自 `src/modules/ekf2/params_external_vision.yaml`，其余来自 `src/modules/ekf2/module.yaml`、`params_gnss.yaml`、`params_magnetometer.yaml`、`params_barometer.yaml`。

EKF2 怎么用这条消息（`src/modules/ekf2/EKF2.cpp::UpdateExtVisionSample` 和 `EKF/aid_sources/external_vision/`）：

- 位置只在 `pose_frame` 为 `NED(1)` 或 `FRD(2)` 时使用；FAST-LIO 世界系航向任意，桥接默认发 **FRD(2)**。
- 四元数必须有限、非零且模长误差 ≤ 1e-5，否则整条姿态被丢弃（桥接节点会先归一化，单测里用 float32 验证过）。
- 速度是 NaN、`velocity_frame` 是 UNKNOWN 时，只是不融合速度（`ev_vel_control.cpp` 要求速度全部有限），位置、高度和偏航照常融合。
- 两条 EV 样本间隔必须 < 200 ms（`EV_MAX_INTERVAL`）才会开始融合，即**至少 5 Hz**；FAST-LIO 是 10 Hz。超过 400 ms 没有数据，EKF2 自动停止 EV 融合。
- **有 GNSS 时 FRD 的 EV 偏航不会被使用**（`ev_yaw_control.cpp`：GNSS 在融合且航向已对准时，只接受 NED 帧的 EV 偏航）。这时 EV 位置会先按"EKF 姿态与 EV 姿态之差"旋转到 EKF 的坐标系，再带一个偏置估计器融合，相当于 GNSS 之外的第二位置源。所以室外建议只开位置（`3`）。
- **没有 GNSS 时**开了偏航位（`11`）：EKF 把自己的航向重置成 EV 的航向，之后直接在 EV 坐标系里工作。PX4 的本地坐标系 x 轴 = FAST-LIO 启动时机头方向，**不是正北**。
- `reset_counter` 变化会让 EKF 直接跳到 EV 给的位姿（无 GNSS 时重置位置和航向，有 GNSS 时只重置 EV 偏置）。桥接节点不会因为单帧突跳就加计数，见 [ryan-rebuild.md](ryan-rebuild.md) 的"不连续的处理"。
- 即使 `reset_counter` 不变，在 EV 是唯一位置源时，EV 位置连续 1 s 通不过新息门限，EKF2 也会自己重置到 EV（空中最多 5 次）。也就是说：**FAST-LIO 稳定地算错时，PX4 最终会相信它。** 能发现这种错误的只有 GNSS 或飞手。

### 高度基准

`EKF2_HGT_REF=3` 让高度跟随 FAST-LIO，室内没有气压漂移，这是 PX4 文档对纯视觉飞行的建议。但 FAST-LIO 的世界系是启动瞬间的 IMU 系，**不做重力对齐**：起飞面有 2° 坡度、或 `imu_to_base_rpy` 量错 2°，水平飞 20 m 就有约 0.7 m 的高度误差，而且在"EV 位置 + EV 偏航"同时启用时 EKF2 不会替你修正（`ev_height_control.cpp`：完全在视觉坐标系里工作时不做旋转）。

所以设成 3 之前要满足：在水平地面上启动 FAST-LIO，并且 `imu_to_base_rpy` 已实测。做不到就把 `EKF2_HGT_REF` 留在气压（0）或 GPS（1），让 EV 高度只作辅助。

## 3. 上机前检查（不涉及起飞）

1. 飞控上电、桨叶拆除，Jetson 跑 `mapping.launch.py`。
2. QGC → Analyze Tools → MAVLink Console：
   - `listener vehicle_visual_odometry`：有数据，`pose_frame: 2`，位置和四元数在变；
   - `listener estimator_status_flags`：`cs_ev_pos`、`cs_ev_hgt`（以及室内配置下的 `cs_ev_yaw`）为 `True`。
   注意 MAVLink Inspector 里的 `ODOMETRY` 是 PX4 自己的估计输出，不能说明 EV 输入到了没有。
3. 手持整机前后左右移动，`listener vehicle_local_position` 的 x/y/z 方向与实际一致（前 = +x，右 = +y，上 = −z）。
4. 原地转 90°：航向变化方向与 QGC 地图一致。
5. 把整机快速挪开 2 m 再放回：`/hummingbird/px4_bridge/status` 的 `glitches` 不应增加（正常运动不是故障）。
6. 上述任何一项方向不对，先改 `imu_to_base_rpy` / `static_tf.yaml`，**不要试飞**。
