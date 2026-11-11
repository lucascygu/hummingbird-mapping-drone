# Ryan 重建：Mid-360 + FAST-LIO2 + PX4 建图工作空间

本文档由 Grok 撰写/协助

> 状态：**由助手根据硬件规格重建，等待 Ryan 审核**。Ryan 原来的代码已丢失，这里不是他的原代码。
> 分支：`ryan-rebuild`。没有上过真机，也没有在 Jetson 上编译过。

## 数据流

```text
Livox Mid-360 ──UDP 192.168.1.x──▶ livox_ros_driver2 ──/livox/lidar (CustomMsg), /livox/imu──▶ FAST-LIO2
FAST-LIO2 ──/Odometry (camera_init→body)──▶ hummingbird_px4_bridge ──/fmu/in/vehicle_visual_odometry──▶ MicroXRCEAgent ──TELEM2──▶ PX4 EKF2
FAST-LIO2 ──/cloud_registered──▶ map_recorder ──/hummingbird/save_map──▶ ~/hummingbird_maps/*.pcd ──pcd_export──▶ 降采样 PCD/PLY/XYZ
任务层：mission_gate 只发 HighLevelSetpoint 请求，目前没有任何节点把它转成 PX4 指令。
```

## 坐标系

| 帧 | 由谁发布 | 含义 |
| --- | --- | --- |
| `map` → `camera_init` | `static_tf.launch.py`（单位阵占位） | 以后空地协同对齐用 |
| `camera_init` → `body` | FAST-LIO2 | `camera_init` = FAST-LIO 启动时的 IMU 帧；`body` = Mid-360 内置 IMU |
| `body` → `livox_frame` | `static_tf.launch.py` | IMU→激光雷达偏移 `[-0.011, -0.02329, 0.04412]`，取自 FAST_LIO_ROS2 的 `config/mid360.yaml` |
| `body` → `base_link` | `static_tf.launch.py` | **占位**，需实测；须与桥接参数 `imu_to_base_xyz/rpy` 一致（有单测检查） |

桥接节点的换算（`hummingbird_px4_bridge/frames.py`，有 pytest）：
1. `T_base0_base(t) = T_base_imu · T_camera_init_body(t) · T_imu_base`：把 FAST-LIO 世界系重新表达成"启动时的机体 FLU 系"，Mid-360 倒装/斜装也能得到正确的前-左-上。
2. FLU → FRD：`p' = (x, −y, −z)`，`R' = F·R·F`（`F = diag(1,−1,−1)`），四元数 `(w, x, −y, −z)`；输出 `POSE_FRAME_FRD`。
3. 若上游世界系是真 ENU（例如以后做了 GNSS 对齐），设 `output_frame: ned`：`p' = (y, x, −z)`，输出 `POSE_FRAME_NED`。
4. 时间戳：`timestamp` = 当前 ROS 时间（µs），`timestamp_sample` = FAST-LIO header 时间；若 header 与 ROS 时间差 > 0.5 s（Mid-360 未做 PTP 同步时会这样），自动改用接收时间并告警。
5. 方差：默认用固定值（位置 0.01 m²，姿态 0.0025 rad²）。原因：FAST_LIO_ROS2 的 `publish_odometry` 把位置/姿态协方差块的顺序对调了，而且是在 `publish()` 之后才写入，每条消息带的是上一帧的协方差。可选 `covariance_source: fastlio` 按对调后的下标读取。
6. 速度：FAST_LIO_ROS2 不填 twist，默认发 NaN（PX4 不融合速度）；可选差分估计。

## 构建

```bash
# Jetson 原生（Ubuntu 22.04 + ROS 2 Humble 假设）
bash scripts/setup_jetson_native.sh

# 或 docker（x86 开发机；Jetson 基础镜像未测试）
docker build -f docker/Dockerfile -t hummingbird:humble .
```

`hummingbird.repos` 把四个 fork 钉在 2026-11-11 核对过的提交上。`px4_msgs` 必须与飞控固件同一 release 分支（这里是 `release/1.15`）。

## 运行

```bash
source ros2_ws/install/setup.bash
ros2 launch hummingbird_bringup mapping.launch.py                 # 雷达 + FAST-LIO + 桥接 + 存图
ros2 launch hummingbird_bringup mapping.launch.py start_xrce_agent:=true xrce_dev:=/dev/ttyTHS1
ros2 launch hummingbird_bringup mapping.launch.py rviz:=true      # 有显示器时
ros2 launch hummingbird_bringup mapping.launch.py with_livox:=false use_sim_time:=true   # 回放 bag
```

存图与导出：

```bash
ros2_ws/src/hummingbird_bringup/scripts/save_map.sh          # 调 /hummingbird/save_map
ros2 run hummingbird_bringup pcd_export ~/hummingbird_maps/hummingbird_map_*.pcd map_10cm.pcd --voxel 0.10
ros2 run hummingbird_bringup pcd_export in.pcd out.ply --voxel 0.05 --crop -50 -50 -5 50 50 30
```

关于 FAST-LIO 自带的存图：这个 ROS 2 移植版里，`/map_save` 只保存 `publish.map_en: true` 时累积的 `/Laser_map` 缓冲（每秒追加一帧、无上限、每秒整包重发），退出时写 `PCD/scans.pcd` 的代码被注释掉了。所以默认 `map_en: false`，改用 `map_recorder`（体素地图，内存有上限）。

PX4 参数见 [px4-params.md](px4-params.md)。

## 安全边界

- 桥接节点只发布 `VehicleOdometry` 到 `/fmu/in/vehicle_visual_odometry`；`test_no_command_paths.py` 静态检查源码里没有 VehicleCommand / Offboard / Trajectory / Actuator 等符号。
- `mapping.launch.py` 不解锁、不切模式、不发 setpoint。
- `mission_gate` 仍然只发 `HighLevelSetpoint` 请求，没有任何消费者把它转给 PX4；要接 offboard 必须单独评审。
- 注意：一旦 PX4 开始融合 EV，错误的外参/方向会直接影响定位和飞行，务必按 px4-params.md 第 3 节做手持检查。

## 待 Ryan 确认

- Jetson 型号（假设 Orin NX/Nano，JetPack 6 / Ubuntu 22.04）、ROS 2 发行版（假设 Humble）
- PX4 固件版本（假设 v1.15.x，决定 `px4_msgs` 分支）
- Mid-360 安装方向和 IMU→机体重心的外参（`imu_to_base_*`、`static_tf.yaml`）
- 雷达 IP（出厂一般为 192.168.1.1XX，XX 为序列号末两位，**未核实**）和 Jetson 网口地址
- TELEM2 接到 Jetson 的哪个 UART（`/dev/ttyTHS*`）
- 是否用 PTP 给 Mid-360 授时

## 验证记录（2026-11-11，助手在云端 Linux x86_64 上做的，不是 Jetson）

已做：
- `scripts/run_tests.sh`：80 个 pytest 全部通过（桥接坐标换算 63、配置/存图工具 12、mission_gate 5）。
- 用 RoboStack（conda）装的 ROS 2 Humble 执行 `colcon build`：`hummingbird_interfaces`、`hummingbird_task_agent`、`hummingbird_bringup`、`hummingbird_px4_bridge`、`px4_msgs`（release/1.15）5 个包编译通过。
- 冒烟测试：向 `/Odometry` 发假数据（位置 (1,2,3)、左转 90°），桥接节点在 `/fmu/in/vehicle_visual_odometry` 输出 `pose_frame=2`、位置 (1,−2,−3)、四元数 (0.7071, 0, 0, −0.7071)（即 FRD 偏航 −90°），符合预期。
- `map_recorder` 收到假 `/cloud_registered` 后，`/hummingbird/save_map` 成功写出 PCD，`pcd_export` 能降采样导出 PLY；`static_tf.launch.py` 发布的 `body→livox_frame` 用 `tf2_echo` 核对过。
- `mapping.launch.py` 能被 `ros2 launch --print` 正常解析。

没做：
- 没有编译 FAST_LIO_ROS2、livox_ros_driver2、Livox-SDK2、Micro-XRCE-DDS-Agent（云端环境没有 PCL / Livox SDK，也没有 docker）。
- `docker/Dockerfile` 和 `scripts/setup_jetson_native.sh` 没有实际跑过。
- 没有连接真实 Mid-360、Pixhawk 6C 或 PX4 SITL；PX4 是否真正融合 EV 未验证。
