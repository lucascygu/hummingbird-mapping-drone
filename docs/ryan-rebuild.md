# Ryan 重建：Mid-360 + FAST-LIO2 + PX4 建图工作空间

本文档由 Grok 撰写/协助；2026-11-11 由 Claude 审阅、修正并补全（改动见文末"审阅记录"）。

> 状态：**由助手根据硬件规格重建，等待 Ryan 审核**。Ryan 原来的代码已丢失，这里不是他的原代码。
> 分支：`ryan-rebuild`。**没有上过真机，也没有在 Jetson 上编译过。**
> 已在云端 x86_64 上把整条链路（假 Mid-360 → FAST-LIO2 → 桥接 / 存图 / 分块）跑通并对照真值打分，见"验证记录"。

## 数据流

```text
Livox Mid-360 ──UDP 192.168.1.x──▶ livox_ros_driver2 ──/livox/lidar (CustomMsg), /livox/imu──▶ FAST-LIO2
FAST-LIO2 ──/Odometry (camera_init→body)──▶ hummingbird_px4_bridge ──/fmu/in/vehicle_visual_odometry──▶ MicroXRCEAgent ──TELEM2──▶ PX4 EKF2
FAST-LIO2 ──/cloud_registered──▶ map_recorder ──自动保存 / /hummingbird/save_map──▶ ~/hummingbird_maps/*.pcd ──pcd_export──▶ 降采样 PCD/PLY/XYZ
FAST-LIO2 ──/cloud_registered──▶ map_tiler（可选）──/hummingbird/map/tile (MapTile 高度分块)──▶ 地面站
地面站 ──RequestMission 服务 / GroundTask 话题──▶ mission_gate ──/hummingbird/task/setpoint_request──▶ （还没有消费者）
```

任务层只到 `HighLevelSetpoint` 为止：**目前没有任何节点把它转成 PX4 指令**，见"还没做的部分"。

## 已从团队报告确认的版本

下列内容来自团队 2026 年 3 月的项目报告，不再是"待确认"：

| 项 | 值 |
| --- | --- |
| 机载计算机 | Jetson Orin Nano Super 8GB，JetPack 6.2（Ubuntu 22.04） |
| ROS 2 | Humble Hawksbill |
| 飞控 / 固件 | Pixhawk 6C，**PX4 Autopilot v1.16.1**；地面站 QGroundControl v5.0.8 |
| 雷达驱动 | Livox SDK2 v1.2.5、livox_ros_driver2 v1.2.4（团队当时用的版本） |
| SLAM | FAST-LIO2 |
| 定位 | NEO 4 SE（DroneCAN GNSS） |
| 遥控 / 数传 | RadioLink T12D（SBUS）、MicroAir LR 900A（UART） |

两点与本仓库不同，需要 Ryan 知情：

1. `hummingbird.repos` 钉住的 fork 比团队当时用的新：Livox-SDK2 是 v1.5.2、livox_ros_driver2 是 v1.2.8。两者都支持 Mid-360，在云端编译并跑通了仿真；如果在 Jetson 上遇到驱动问题，先退回团队验证过的 v1.2.5 / v1.2.4。
2. 报告里写的 Pixhawk↔Jetson 链路是 **MAVLink over UART**，而且最终接线图里**这条线还没有接**。本仓库走的是 PX4 官方推荐给 ROS 2 的 **uXRCE-DDS**（TELEM2）。两条路都可行，但只能二选一占用 TELEM2；要坚持 MAVLink 的话应改用 MAVROS 的里程计输入，本仓库的桥接节点就不适用了。

## 坐标系

| 帧 | 由谁发布 | 含义 |
| --- | --- | --- |
| `map` → `camera_init` | `static_tf.launch.py`（单位阵占位） | 以后空地协同对齐用 |
| `camera_init` → `body` | FAST-LIO2 | `camera_init` = FAST-LIO 启动时的 IMU 帧；`body` = Mid-360 内置 IMU |
| `body` → `livox_frame` | `static_tf.launch.py` | IMU→激光雷达偏移 `[-0.011, -0.02329, 0.04412]`，取自 FAST_LIO_ROS2 的 `config/mid360.yaml` |
| `body` → `base_link` | `static_tf.launch.py` | **占位**，需实测；须与桥接参数 `imu_to_base_xyz/rpy` 一致（有单测检查） |

桥接节点的换算（`hummingbird_px4_bridge/frames.py`、`odom_conversion.py`，有 pytest）：

1. `T_base0_base(t) = T_base_imu · T_camera_init_body(t) · T_imu_base`：把 FAST-LIO 世界系重新表达成"启动时的机体 FLU 系"，Mid-360 倒装/斜装也能得到正确的前-左-上。
2. FLU → FRD：`p' = (x, −y, −z)`，`R' = F·R·F`（`F = diag(1,−1,−1)`），四元数 `(w, x, −y, −z)`；输出 `POSE_FRAME_FRD`。
3. 若上游世界系是真 ENU（例如以后做了 GNSS 对齐），设 `output_frame: ned`：`p' = (y, x, −z)`，输出 `POSE_FRAME_NED`。
4. 时间戳：`timestamp` = 当前 ROS 时间（µs），`timestamp_sample` = FAST-LIO header 时间；若 header 与 ROS 时间差 > 0.5 s（Mid-360 未做 PTP 同步时会这样），改用接收时间并告警。这个选择是**粘滞**的：要连续 20 帧都指向另一种时间源才切换，不会逐帧来回跳（来回跳会让时间序列前后错位，EKF2 当成乱序数据丢掉）。
5. 方差：默认用固定值（位置 0.01 m²，姿态 0.0025 rad²）。原因：FAST_LIO_ROS2 的 `publish_odometry` 把位置/姿态协方差块的顺序对调了，而且是在 `publish()` 之后才写入，每条消息带的是上一帧的协方差。可选 `covariance_source: fastlio` 按对调后的下标读取；非固定方差会按安装旋转换到机体轴上。
6. 速度：FAST_LIO_ROS2 不填 twist，默认发 NaN（PX4 不融合速度）；可选差分估计。
7. **不连续的处理**。PX4 一旦看到 `reset_counter` 变化，就会把估计值直接跳到外部视觉给的位姿，而不是用新息门限去检验它。所以桥接节点绝不为"看起来不合理的一帧"加计数，否则 FAST-LIO 发散时会把飞控一起带偏：
   - 时间戳倒退或重复的帧直接丢弃；
   - 位置突跳（步长 > `reset_jump_m` 且等效速度 > `max_plausible_speed_mps`，默认 1 m、30 m/s）视为故障：**停发** `holdoff_s`（默认 1 s，长于 EKF2 的 0.4 s 视觉超时），数据稳定后才以新会话恢复（`reset_counter`+1）。只要还在跳，就一帧都不发；
   - 断流超过 `gap_reset_s`（默认 1 s）或机载时钟回拨，恢复时按新会话处理。
   - 正常飞行中的丢帧（例如 0.6 s 没数据、期间飞了 3 m）不会被误判为重置。
8. 健康状态：每秒在 `/hummingbird/px4_bridge/status` 发一条 JSON（`state`: waiting / ok / holdoff / stale，输入频率、时间源、各类丢帧计数、PX4 端是否有订阅者）。FAST-LIO 停止输出 1 s 后会报错。

**已知限制：世界系不保证水平。** FAST-LIO 的世界系就是启动瞬间的 IMU 系，不做重力对齐。起飞面有 2° 坡度（或 `imu_to_base_rpy` 量错 2°）时，水平飞 20 m 会带来约 0.7 m 的高度误差。在 PX4 同时用 EV 位置和 EV 偏航、且把 EV 设为高度基准时，这个误差会直接进入高度估计。对策见 [px4-params.md](px4-params.md)：高度基准留给气压计 / GNSS，并在水平地面上启动 FAST-LIO。

## 构建

```bash
# Jetson 原生（Ubuntu 22.04 + ROS 2 Humble）
bash scripts/setup_jetson_native.sh

# 或 docker（x86 开发机；Jetson 基础镜像未测试）
docker build -f docker/Dockerfile -t hummingbird:humble .
```

`hummingbird.repos` 把三个 fork（FAST_LIO_ROS2、livox_ros_driver2、px4_msgs）钉在 2026-11-11 核对过的提交上。`px4_msgs` 必须与飞控固件同一 release 分支：固件是 v1.16.1，所以钉在 `release/1.16`。

`px4_ros_com` 已从清单里拿掉：本工作空间用不到它，而它会装上 `offboard_control`、`offboard_control_srv`、`offboard_control.py` 三个示例程序，**一运行就解锁并飞到 5 m**。把它装在机载电脑上，等于离无人看管的起飞只差一条敲错的 `ros2 run`。

## 运行

```bash
source ros2_ws/install/setup.bash
ros2 launch hummingbird_bringup mapping.launch.py                 # 雷达 + FAST-LIO + 桥接 + 存图
ros2 launch hummingbird_bringup mapping.launch.py start_xrce_agent:=true xrce_dev:=/dev/ttyTHS1
ros2 launch hummingbird_bringup mapping.launch.py rviz:=true      # 有显示器时
ros2 launch hummingbird_bringup mapping.launch.py with_map_tiler:=true with_mission_gate:=true
ros2 launch hummingbird_bringup mapping.launch.py with_livox:=false use_sim_time:=true   # 回放 bag
```

存图与导出：

```bash
ros2_ws/src/hummingbird_bringup/scripts/save_map.sh          # 调 /hummingbird/save_map
ros2 run hummingbird_bringup pcd_export ~/hummingbird_maps/hummingbird_map_*.pcd map_10cm.pcd --voxel 0.10
ros2 run hummingbird_bringup pcd_export in.pcd out.ply --voxel 0.05 --crop -50 -50 -5 50 50 30
```

地图只存在内存里，所以 `map_recorder` 不等人来调服务也会写盘：

- 每 `map_autosave_s`（默认 60 s）滚动写一次 `hummingbird_map_autosave.pcd`，崩溃或电池耗尽最多丢一个周期；
- 退出时（Ctrl-C / 停掉 launch）再写一个带时间戳的文件；
- 所有写盘都是"先写临时文件再改名"，中途断电不会留下半截文件覆盖掉上一份好图。
- 内存：每个体素约 50 字节，合并时短暂达到约 3 倍。默认上限 1000 万体素（常驻约 0.5 GB），Jetson 吃紧时调小 `map_max_points`。

`fast_lio_mid360.yaml` 里 `dense_publish_en` 已改为 `true`：上游默认只发布经 0.5 m 体素滤波后的每帧点云，存下来的地图非常稀；改为 `true` 后 `/cloud_registered` 是 FAST-LIO 实际保留的全部点，这才是要交付的建图结果。

关于 FAST-LIO 自带的存图：这个 ROS 2 移植版里，`/map_save` 只保存 `publish.map_en: true` 时累积的 `/Laser_map` 缓冲（每秒追加一帧、无上限、每秒整包重发），退出时写 `PCD/scans.pcd` 的代码被注释掉了。所以默认 `map_en: false`，改用 `map_recorder`（体素地图，内存有上限）。

PX4 参数见 [px4-params.md](px4-params.md)。

## 没有硬件时怎么跑

`hummingbird_bringup/mid360_sim` 是一个假 Mid-360：按已知轨迹在一个已知房间里飞一圈，发布与真驱动相同的 `/livox/lidar`（CustomMsg，逐点时间戳）和 `/livox/imu`（200 Hz，加速度单位为 g），同时发布真值 `/sim/ground_truth`。真正的 FAST-LIO2、桥接、存图、分块节点原样运行。

```bash
ros2 launch hummingbird_bringup sim_mapping.launch.py                    # 只跑
bash scripts/sim_e2e_test.sh                                             # 跑 + 打分，退出码 0 = 通过
bash scripts/sim_e2e_test.sh 46 lidar_clock:=boot                        # 模拟雷达时钟未同步
```

它是测试信号发生器，不是飞行仿真：没有机体动力学，也没有 PX4。它能证明"建图链路和桥接输出是对的"，不能证明"PX4 融合后飞得稳"。

## 安全边界

- 桥接节点只发布 `VehicleOdometry` 到 `/fmu/in/vehicle_visual_odometry`（外加一个健康状态字符串话题）；`test_no_command_paths.py` 静态检查源码里没有 VehicleCommand / Offboard / Trajectory / Actuator 等符号。
- `mapping.launch.py` 不解锁、不切模式、不发 setpoint。
- `mission_gate` 仍然只发 `HighLevelSetpoint` 请求，没有任何消费者把它转给 PX4；要接 offboard 必须单独评审。
- 注意：一旦 PX4 开始融合 EV，错误的外参/方向会直接影响定位和飞行，务必按 px4-params.md 第 3 节做手持检查。
- **不要在飞行中重启 FAST-LIO。** 重启后它的世界系原点和朝向都变成重启瞬间的机体位姿；桥接会按新会话恢复，PX4 的本地坐标系随之改变，之前的航点和返航点全部失效。

## 还没做的部分

- **`HighLevelSetpoint` → PX4 的执行器**（offboard 跟踪）。这是飞行控制代码，在没有 SITL 闭环验证之前不应该写进来；Grok 当时有意留空，这次也没有补。
- **PX4 SITL 里验证 EV 融合**。桥接的消息格式、话题名、QoS、时间戳处理都对照 PX4 v1.16.1 源码核对过，但没有真正接到一个运行中的 PX4 上。
- Jetson 上的实际编译、`docker/Dockerfile`、`scripts/setup_jetson_native.sh` 的实跑。
- 一切需要真机的东西：外参标定、雷达 IP、UART 设备名、PTP 授时。

## 待 Ryan 确认

- Mid-360 安装方向和 IMU→机体重心的外参（`imu_to_base_*`、`static_tf.yaml`）
- 雷达 IP（出厂一般为 192.168.1.1XX，XX 为序列号末两位，**未核实**）和 Jetson 网口地址
- TELEM2 接到 Jetson 的哪个 UART（`/dev/ttyTHS*`），以及是否接受把 Pixhawk↔Jetson 链路从报告里的 MAVLink 换成 uXRCE-DDS
- 是否用 PTP 给 Mid-360 授时
- 试飞场地的 geofence（`hummingbird_task_agent/config/mission_gate.yaml` 里全是占位数值）

## 验证记录

### 2026-11-11，Claude，云端 Linux x86_64（Ubuntu 24.04 + ROS 2 **Jazzy**，不是 Jetson，也不是 Humble）

已做：

- `scripts/run_tests.sh`：纯 Python 单测 172 个全部通过（桥接 87、建图工具与仿真 29、任务门控 56，另有 2 个文件在没有 ROS 时自动跳过）。
- 在已构建的工作空间里另有 9 个进程内 ROS 测试通过：`mission_gate` 节点 5 个（服务、GroundTask、批准流程、按序发布）、桥接节点 4 个（对照真实的 `px4_msgs` release/1.16、PX4 端的 BEST_EFFORT QoS）。
- `colcon build`：`hummingbird_interfaces`、`hummingbird_task_agent`、`hummingbird_bringup`、`hummingbird_px4_bridge`、`px4_msgs`（release/1.16）、**`livox_ros_driver2`、`fast_lio`（钉住的 fork 提交）**、Livox-SDK2 全部编译通过。
- **端到端仿真**（`scripts/sim_e2e_test.sh`，46 s）：
  - FAST-LIO 位姿对真值：位置 RMSE 1.7 cm，最大 2.8 cm，偏航 RMSE 0.03°（467 帧）；
  - 桥接输出 454 帧，全部是 `POSE_FRAME_FRD`，与 FAST-LIO 位姿的 FLU→FRD 换算一致，`reset_counter` 全程为 0；
  - 存下的地图 35.7 万点，99% 的点离房间真实表面 ≤ 5.3 cm（仿真测距噪声 2 cm）；
  - 高度分块正常发布；雷达时钟未同步的情形（`lidar_clock:=boot`）也通过，桥接按预期退回接收时间。
- 停止流程：Ctrl-C 后各节点干净退出，`map_recorder` 在退出时写出了带时间戳的地图。

没做：

- 没有在 ROS 2 Humble 上重新编译（上面全部是 Jazzy）。代码没有用 Jazzy 独有的接口，但 Humble 上的首次编译仍需在 Jetson 或 Humble 容器里确认。
- 没有编译 Micro-XRCE-DDS-Agent，没有连接 PX4 SITL：**PX4 是否真正融合 EV 仍未验证。**
- `docker/Dockerfile` 和 `scripts/setup_jetson_native.sh` 没有实际跑过。
- 没有连接真实 Mid-360、Pixhawk 6C。

### 2026-11-11，Grok，云端 Linux x86_64

- 80 个 pytest 通过；用 RoboStack（conda）的 ROS 2 Humble 编译了 5 个自有包和 `px4_msgs`（当时是 release/1.15）。
- 向 `/Odometry` 发假数据的冒烟测试、`map_recorder` 存图、`static_tf` 核对、`mapping.launch.py` 解析。

## 审阅记录（Claude，2026-11-11）

改正的问题：

1. **PX4 版本假设错了。** 原来按 v1.15.x 写，实际固件是 v1.16.1。`px4_msgs` 改钉 `release/1.16`；[px4-params.md](px4-params.md) 逐条按 v1.16.1 源码重核（其中 `EKF2_EV_CTRL` 的默认值在 1.16 是 0，不是原文写的 15）。`VehicleOdometry` 的字段两版相同，所以桥接本身碰巧不受影响，但其余 71 个消息定义不同，而且 1.16 给带版本号的话题加了后缀（如 `/fmu/out/vehicle_status_v1`）。
2. **任务门控会挡住返航和降落**（违反 task-agent-design.md 的第 5 条）。原来 `rtl` / `land` 没有 `arm_approval`、或 `frame_id` 为空 / 不在白名单时会被拒绝。现在这两类请求无条件放行。
3. **请求可以把 geofence 放大。** 原来请求里填一个很大的半径 / 高度就能覆盖站点配置。现在请求只能收紧，不能放宽。
4. **NaN / 无穷大航点会通过校验**（NaN 与任何上限比较都是 False）。现在显式拒绝。
5. **桥接的重置逻辑有两个隐患**：不看时间间隔，丢帧后容易误判；更重要的是位置突跳会立刻加 `reset_counter`，等于让 PX4 跟着发散的 FAST-LIO 走。见上文第 7 条。
6. **地图只在有人调服务时才写盘**，Ctrl-C 或崩溃就丢图。并且在 `ros2 launch` 下 Ctrl-C 会给每个节点发两次 SIGINT，第二次会打断退出时的清理。现在信号只置标志位，清理一定跑完。
7. `/cloud_registered` 只有 0.5 m 降采样后的稀疏点云，存出来的地图不能用；改为发布稠密点云。
8. `map_voxel:=1` 这类整数写法会因参数类型不符让节点启动失败；launch 里改成显式类型。
9. **依赖清单会把能解锁起飞的示例程序装到机上。** `px4_ros_com` 的 offboard 示例一运行就解锁；已从 `hummingbird.repos` 移除（见"构建"）。

补全的部分（文档里写了但没有实现）：

- `GroundTask` 话题入口，以及它需要的"草案 + 人工批准"流程（`ApproveMission` 服务）；
- `HighLevelSetpoint` 带上动作类型、`sequence` / `total`、偏航和速度上限，`rtl` / `land` 也有对应消息；
- 站点 geofence、原点、速度上限做成 `mission_gate` 的参数和配置文件；
- `MapTile` 发布节点（`map_tiler`，高度分块）；
- 桥接健康状态话题；
- 假 Mid-360、真值打分工具、端到端测试脚本；
- GitHub Actions 持续集成（`.github/workflows/ci.yml`）：纯 Python 单测；在 `ros:humble` 容器里编译整套依赖、跑节点测试和端到端仿真；Docker 镜像构建为手动触发。
