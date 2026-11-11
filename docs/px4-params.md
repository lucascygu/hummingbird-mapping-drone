# PX4 参数：FAST-LIO2 外部视觉（EV）融合 + TELEM2 uXRCE-DDS

> 适用：Pixhawk 6C + PX4 **v1.15.x**（参数名逐条对照 fork [lucascygu/PX4-Autopilot](https://github.com/lucascygu/PX4-Autopilot) 的 `release/1.15` 分支源码核对，见每行"出处"）。
> 其它版本（v1.14 / v1.16+）参数可能有增删，**刷固件后请在 QGC 里逐个确认**。
> 本文件只涉及估计器输入和通信，**不包含任何解锁、起飞或 offboard 设置**。

## 1. TELEM2 ↔ Jetson（uXRCE-DDS 串口）

| 参数 | 建议值 | 默认值（1.15） | 说明 / 出处 |
| --- | --- | --- | --- |
| `MAV_1_CONFIG` | `0`（Disabled） | 实例 1 默认为空（禁用） | 确保 TELEM2 上没有 MAVLink 抢占串口。`src/modules/mavlink/module.yaml`：`default: [TEL1, "", ""]` |
| `UXRCE_DDS_CFG` | `102`（TELEM2） | 0 | 端口编号来自 `Tools/serial/generate_config.py`：`"TEL2": index 102`；参数定义在 `src/modules/uxrce_dds_client/module.yaml`（`port_config_param: UXRCE_DDS_CFG`） |
| `SER_TEL2_BAUD` | `921600` | 921600 | `Tools/serial/generate_config.py` 中 TEL2 `default_baudrate: 921600`；参数模板 `Tools/serial/serial_params.c.jinja` |
| `UXRCE_DDS_SYNCT` | `1` | 1 | 时间同步：client 会把 `timestamp` / `timestamp_sample` 减去 agent↔飞控时钟差（`Tools/msg/templates/ucdr/msg.h.em`） |
| `UXRCE_DDS_DOM_ID` | `0` | 0 | 与 Jetson 上 `ROS_DOMAIN_ID` 一致 |
| `UXRCE_DDS_KEY` | `1` | 1 | 单飞控单 agent 保持默认 |

板级串口映射：Pixhawk 6C 的 TELEM2 = `/dev/ttyS3`（`boards/px4/fmu-v6c/default.px4board`：`CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS3"`）。

Jetson 端（agent 必须是 **v2.x**，PX4 文档说明 v3.x agent 与 PX4 client 不兼容）：

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600   # 设备名取决于载板，未在实机核对
# 或： ros2 launch hummingbird_bringup mapping.launch.py start_xrce_agent:=true xrce_dev:=/dev/ttyTHS1
```

接线：TELEM2 TX→Jetson RX、RX→TX、GND→GND（3.3 V 电平）。**不要接 TELEM2 的 5 V 到 Jetson。**
验证：`ros2 topic list | grep /fmu/` 应能看到 `/fmu/in/vehicle_visual_odometry`（订阅端在 `src/modules/uxrce_dds_client/dds_topics.yaml` 第 136 行）。

## 2. EKF2 外部视觉融合

| 参数 | 建议值 | 默认值（1.15） | 说明 |
| --- | --- | --- | --- |
| `EKF2_EV_CTRL` | `11`（bit0 水平位置 + bit1 垂直位置 + bit3 偏航） | 15 | bit 定义：0 水平位置 / 1 垂直位置 / 2 三维速度 / 3 偏航。桥接节点默认**不发速度**（FAST_LIO_ROS2 不填 twist），所以关 bit2；若开启 `estimate_velocity:=true` 再改成 15 |
| `EKF2_HGT_REF` | `3`（Vision）——室内/无 GPS 建图；室外有 GPS 时保留 `1` | 1 | 枚举：0 气压 / 1 GPS / 2 测距 / 3 视觉。**需重启** |
| `EKF2_EV_DELAY` | 先 `0`（Mid-360 已 PTP 同步、桥接用 header 时间）；若桥接退回接收时间（日志会告警），从 `50` 起调 | 0 | 单位 ms，0–300，**需重启**。EKF 用 `timestamp_sample - EV_DELAY` 作为测量时刻（`EKF/estimator_interface.cpp`） |
| `EKF2_EV_NOISE_MD` | `0` | 0 | 0 = 用消息里的方差、参数作下限；1 = 直接用参数 |
| `EKF2_EVP_NOISE` | `0.1` m | 0.1 | 位置噪声下限（最小 0.01） |
| `EKF2_EVA_NOISE` | `0.1` rad | 0.1 | 角度噪声下限（最小 0.05） |
| `EKF2_EV_QMIN` | `0` | 0 | 桥接节点发 `quality=0`（未知），QMIN=0 时不检查 |
| `EKF2_EV_POS_X/Y/Z` | **`0`** | 0 | 桥接节点已用 `imu_to_base_xyz` 把位姿换算到 base_link（机体重心）。**如果这里再填杠杆臂，会重复补偿** |
| `EKF2_BARO_CTRL` | `1`（保留，气压作辅助） | 1 | |
| `EKF2_GPS_CTRL` | 室内：`0`；室外：默认 `7` | 7 | |
| `EKF2_MAG_TYPE` | 室内（钢结构干扰大）可设 `5`（None），由 EV 偏航提供航向 | 0 | **需重启**。室外有 GPS 时保留默认 |

上表 `EKF2_*` 全部来自 `src/modules/ekf2/module.yaml`（release/1.15）。EV 消息解析逻辑见 `src/modules/ekf2/EKF2.cpp::UpdateExtVisionSample`：
- 位置只在 `pose_frame` 为 `NED(1)` 或 `FRD(2)` 时使用；FAST-LIO 世界系航向任意，桥接默认发 **FRD(2)**。
- 四元数必须有限、非零且模长误差 ≤ 1e-5，否则整条姿态被丢弃（桥接节点会先归一化，单测里用 float32 验证过）。
- `reset_counter` 变化会触发 EKF 重置 EV 参考；桥接节点在位置跳变 > `reset_jump_m` 或时间倒退时自增。

## 3. 上机前检查（不涉及起飞）

1. 飞控上电、桨叶拆除，Jetson 跑 `mapping.launch.py`。
2. QGC → MAVLink Inspector：`VISION_POSITION_ESTIMATE` / `ODOMETRY` 有数据；或 MAVLink 控制台 `listener vehicle_visual_odometry`。
3. 手持整机前后左右移动，`listener vehicle_local_position` 的 x/y/z 方向与实际一致（前 = +x，右 = +y，上 = −z）。
4. 原地转 90°：航向变化方向与 QGC 地图一致。
5. 上述任何一项方向不对，先改 `imu_to_base_rpy` / `static_tf.yaml`，**不要试飞**。
