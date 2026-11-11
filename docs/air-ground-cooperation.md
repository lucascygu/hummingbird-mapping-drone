# 空地协同接口草图 Air–Ground Cooperation

本文是接口草图，用于地面站（人或地面机器人）与蜂鸟空中节点协同测绘 / 侦察。消息定义见 `hummingbird_interfaces`。这些话题名是约定，**尚未与实机或 Ryan 的重建代码联调**。

## 角色

- **空中节点**：Jetson 上的 ROS 2 进程 + PX4。负责感知、建图、执行已放行的高层 setpoint。
- **地面节点**：任务下发、地图拼接、人工确认解锁 / 起飞、紧急终止。
- **传输**：高带宽链路走 ROS 2 消息（地图、点云摘要）；低带宽链路只走短指令和心跳，作为备份。

## ROS 2 话题（规划）

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/hummingbird/ground/task` | `hummingbird_interfaces/GroundTask` | 地 → 空 | 一次高层任务（探索、分区测绘、观察、悬停、返航、降落） |
| `/hummingbird/map/tile` | `hummingbird_interfaces/MapTile` | 空 → 地 | 地图分块，避免默认下发全分辨率点云 |
| `/hummingbird/task/setpoint_request` | `hummingbird_interfaces/HighLevelSetpoint` | 门控 → 导航 / 飞控桥 | **唯一**允许由任务层发出的运动请求 |
| `/hummingbird/status` | `std_msgs/String`（后续可换成专用 msg） | 空 → 地 | 门控状态、拒绝原因、当前任务 ID；骨架阶段先用字符串 |

相关内部数据（实现时再接到具体包，不在本仓库伪造驱动）：

- Mid-360 点云：通常为 `livox_ros_driver2` 的 `/livox/lidar`（以驱动配置为准）
- FAST-LIO2 里程计 / 地图：以 `FAST_LIO_ROS2` 的实际话题为准
- PX4：`px4_msgs` 中的位姿、状态、故障保护；由导航层或桥接节点订阅，**不由 agent 直连 actuator 话题**

## ROS 2 服务

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/hummingbird/mission/request` | `hummingbird_interfaces/RequestMission` | 提交任务。`arm_approval` / `takeoff_approval` 必须是这次请求里的显式布尔值；缺省视为未批准 |

服务响应包含 `accepted` 与 `reason`。未批准解锁时，即便任务其余字段合法，也不得进入 armed 飞行。

## 地图共享

- 使用 `MapTile`：`tile_id`、`frame_id`、分辨率、原点、宽高、`data`（占用或高度摘要）。
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

- 具体 QoS（地图用 BEST_EFFORT，任务用 RELIABLE 是合理起点，未验证）。
- 加密与身份（后续再定，不在本骨架实现）。
- 多机编队的时间同步与空域分配（超出当前接口）。
