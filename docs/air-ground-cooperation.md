# 空地协同接口草图 Air–Ground Cooperation

本文档由 Grok 撰写/协助；2026-11-11 由 Claude 按实际实现更新。

本文是接口草图，用于地面站（人或地面机器人）与蜂鸟空中节点协同测绘 / 侦察。消息定义见 `hummingbird_interfaces`。下表的话题和服务在 `ryan-rebuild` 分支上都已有节点实现，并在仿真里跑过；**尚未与实机或真实的地面节点联调**。

## 角色

- **空中节点**：Jetson 上的 ROS 2 进程 + PX4。负责感知、建图、执行已放行的高层 setpoint。
- **地面节点**：任务下发、地图拼接、人工确认解锁 / 起飞、紧急终止。
- **传输**：高带宽链路走 ROS 2 消息（地图、点云摘要）；低带宽链路只走短指令和心跳，作为备份。

## ROS 2 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/hummingbird/ground/task` | `hummingbird_interfaces/GroundTask` | 地 → 空 | 一次高层任务（探索、分区测绘、观察、悬停、返航、降落）。不带批准位：`mission_gate` 先挂起，等人工批准；返航 / 降落立即放行 |
| `/hummingbird/map/tile` | `hummingbird_interfaces/MapTile` | 空 → 地 | 地图分块，避免默认下发全分辨率点云。由 `hummingbird_bringup/map_tiler` 发布（`with_map_tiler:=true`） |
| `/hummingbird/task/setpoint_request` | `hummingbird_interfaces/HighLevelSetpoint` | 门控 → 导航 / 飞控桥 | **唯一**允许由任务层发出的运动请求；按序发布，带 `sequence` / `total` 和动作类型。**目前没有消费者** |
| `/hummingbird/status` | `std_msgs/String`（后续可换成专用 msg） | 空 → 地 | 门控的每次裁决，一条 JSON：`event`（accepted / refused / pending / expired）、`source`、`mission_id`、`code`、`reason`、`setpoints` |
| `/hummingbird/px4_bridge/status` | `std_msgs/String` | 空 → 地 | 里程计桥接的健康状态，每秒一条 JSON（ok / holdoff / stale、输入频率、丢帧计数） |

相关内部数据（实现时再接到具体包，不在本仓库伪造驱动）：

- Mid-360 点云：通常为 `livox_ros_driver2` 的 `/livox/lidar`（以驱动配置为准）
- FAST-LIO2 里程计 / 地图：以 `FAST_LIO_ROS2` 的实际话题为准
- PX4：`px4_msgs` 中的位姿、状态、故障保护；由导航层或桥接节点订阅，**不由 agent 直连 actuator 话题**

## ROS 2 服务

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/hummingbird/mission/request` | `hummingbird_interfaces/RequestMission` | 提交任务。`arm_approval` / `takeoff_approval` 必须是这次请求里的显式布尔值；缺省视为未批准 |
| `/hummingbird/mission/approve` | `hummingbird_interfaces/ApproveMission` | 人工批准一份由 `GroundTask` 挂起的草案。一次批准只对这一份草案有效；草案超时作废。只应由操作员界面或硬件开关调用 |
| `/hummingbird/save_map` | `std_srvs/Trigger` | 让 `map_recorder` 立刻把体素地图写成带时间戳的 PCD |

服务响应包含 `accepted` 与 `reason`。未批准解锁时，即便任务其余字段合法，也不得进入 armed 飞行。

## 地图共享

- 使用 `MapTile`：`tile_id`、`frame_id`、分辨率、原点、宽高、`data`（占用或高度摘要）。
- 已实现的是高度分块（`data_kind: "height"`）：`data[row*width+col]` 是该格子里见过的最高点，单位厘米、相对 `origin_z`，`-32768` 表示没观测到；默认一块 64×64 格、每格 0.25 m（16 m 见方，8 kB），每秒最多发 8 块。室内要把 `max_z` 设到天花板以下，否则整张图都是天花板高度。
- 发送顺序：先发有变化的块，余量用来轮流重发旧块，所以晚加入或丢了包的地面节点最终也能拿到全图。同一个 `tile_id` 的新块覆盖旧块。
- 地面节点按 `frame_id` + 原点把分块拼进自己的全局地图。
- 密集点云只在调试或短距高带宽时按需开启，不作为默认协同载荷。
- 分块应带时间戳与坐标系，过期或错系的分块必须丢弃，不能默默对齐。

## 坐标系对齐 Frame alignment

1. 空中与地面约定同一个世界系名称（建议 `hummingbird_map`），并记录它相对 FAST-LIO2 `map` 以及 PX4 local 原点的关系。
2. 每次任务在 `GroundTask` / `RequestMission` 中携带参考原点或明确 `frame_id`。
3. 做 ENU（ROS）与 NED（PX4）转换的节点要单一、可测试，不要在 agent 里随手改轴。
4. GNSS 可用时，用地理锚点约束地图原点；GNSS 丢失时，协同退化为相对地图，并在状态里标明「未全局锚定」。
5. 外参（Mid-360 相对机体、飞控相对机体）属于标定产物，应随 Ryan 的重建代码一起放入 `src/` 或配置目录，本文不编造数值。

## 通信链路选项

| 选项 | 适合的载荷 | 备注 |
| --- | --- | --- |
| Wi-Fi / 机载热点 / 网状网 | MapTile、压缩点云、服务调用 | 近距测绘的首选高带宽路径；要规划信道与功耗 |
| 蜂窝（LTE / 5G） | 任务、稀疏地图、远程监控 | 依赖覆盖与公网延迟；注意密钥与防火墙，不要把飞控端口暴露到公网 |
| 数传电台 / SiK 等遥测 | 心跳、模式、紧急 RTL / 降落、短任务 ID | 备份低带宽链路；**不够**传地图分块 |
| 有线调试（实验室） | 全部 | 只用于地面调试，不是飞行协同方案 |

建议：任务与安全指令在高带宽链路上用 ROS 2 服务；同时在低带宽链路上保留「返航 / 降落 / 终止」短指令。两条路径都应进入同一门控逻辑，而不是绕过 `mission_gate`。

## 尚未确定

- QoS：目前地图分块和任务都用 RELIABLE（RELIABLE 的发布者能同时匹配可靠和尽力而为的订阅者，反过来不行）。弱链路上是否改成 BEST_EFFORT 加重发，要等实测。
- 加密与身份（后续再定，不在本骨架实现）。
- 多机编队的时间同步与空域分配（超出当前接口）。
