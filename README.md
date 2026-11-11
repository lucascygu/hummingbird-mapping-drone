# 蜂鸟 Hummingbird Mapping Drone

> 本项目文档与设计由 Grok 协助整理 （Documentation and design assisted by Grok）

面向 **具身智能空中节点**（embodied-intelligence aerial node）的三维建图无人机项目，命名取自「蜂鸟 / Hummingbird」，也与「猫大虫」相关设想一脉相承。

目标硬件与软件栈：

- 飞控：Holybro **Pixhawk 6C** + **PX4 v1.16.1**
- 机载计算机：**NVIDIA Jetson Orin Nano Super 8GB**（JetPack 6.2 / Ubuntu 22.04）
- 激光雷达：**Livox Mid-360**
- 中间件：**ROS 2 Humble**
- 激光惯性里程计 / 建图：**FAST-LIO2**（ROS 2 移植：`FAST_LIO_ROS2`）

本仓库当前提供架构文档、空地协同接口草图、任务层 agent 设计，以及一组最小的 ROS 2 接口包与安全门控节点骨架。它**尚未在实机上测试**，也还不是完整飞控 / 建图固件。

## 关于 Ryan 的原项目：代码已丢失，这是全新重建

Ryan 原有的 3D Mapping Drone 工程代码已全部丢失（他曾在一夜间重建过一版，但没有可导入的仓库或存档）。因此：

- 本仓库是 **从零开始的新鲜重建（fresh rebuild）**，不包含 Ryan 的历史提交或源码。
- 下文的硬件清单、分层架构和接口是按既定方案整理的设计稿与骨架，**不要把它理解成已经从原机导入并验证过的代码**。

### 如何提交重建代码（给 Ryan）

任选一种方式即可，推荐两者一起用：

1. **推送到 `ryan-rebuild` 分支**（推荐保留完整工程与提交历史）

   ```bash
   git clone https://github.com/lucascygu/hummingbird-mapping-drone.git
   cd hummingbird-mapping-drone
   git checkout -b ryan-rebuild
   # 把重建后的工作空间、启动文件、标定和配置放进来
   # 建议主要代码放在 src/ 或 ros2_ws/src/ 下，并更新本 README
   git add -A
   git commit -m "Import Ryan rebuild of the mapping drone workspace"
   git push -u origin ryan-rebuild
   ```

   需要写权限。若还不是 collaborator，先请仓库管理员邀请。

2. **直接放到 `src/` 目录再开 PR / 推送**

   仓库预留了 [`src/`](src/) 作为「Ryan 重建代码放置处」。可以把 PX4 配置、ROS 2 工作空间摘录、标定文件、启动脚本等放到该目录（或在其中按包拆分），并在 [`src/README.md`](src/README.md) 里说明目录结构。不要覆盖 `ros2_ws/src/hummingbird_*` 这套新建的接口骨架，除非是有意合并。

`ryan-rebuild` 分支已创建，并已放入助手重建的工作空间（见下节）。`main` 上的文档和 `hummingbird_*` 包可以并存，不必互相覆盖。

## Ryan 重建

> **这是助手根据硬件规格（Pixhawk 6C / PX4 v1.16.1、Jetson Orin Nano Super、Livox Mid-360、ROS 2 Humble、FAST-LIO2）从零重建的版本，不是 Ryan 的原代码，等待 Ryan 审核。** 没有上过真机，也没有在 Jetson 上编译过。
>
> 2026-11-11 Claude 审阅并补全了这一版：修正了 PX4 版本假设（原按 v1.15 写）、任务门控会挡住返航/降落等问题，并用一个假 Mid-360 把 FAST-LIO2 → 桥接 → 存图整条链路在云端跑通、对照真值打分。改了什么、验证了什么、还缺什么，见 [docs/ryan-rebuild.md](docs/ryan-rebuild.md) 末尾。

详细说明见 [docs/ryan-rebuild.md](docs/ryan-rebuild.md)，PX4 参数见 [docs/px4-params.md](docs/px4-params.md)。

| 内容 | 路径 |
| --- | --- |
| vcstool 清单（钉住 lucascygu 下的 FAST_LIO_ROS2 / livox_ros_driver2 / px4_msgs 提交；`px4_ros_com` 已从清单移除，因为它自带的 offboard 示例会直接解锁起飞） | [`ros2_ws/hummingbird.repos`](ros2_ws/hummingbird.repos) |
| 启动文件、Mid-360 网络配置、FAST-LIO2 配置、静态 TF、RViz、存图/导出工具 | [`ros2_ws/src/hummingbird_bringup`](ros2_ws/src/hummingbird_bringup) |
| FAST-LIO `/Odometry`（ENU/FLU）→ PX4 `VehicleOdometry`（NED/FRD）桥接，**只喂估计器、不解锁** | [`ros2_ws/src/hummingbird_px4_bridge`](ros2_ws/src/hummingbird_px4_bridge) |
| Docker（ros:humble + Livox-SDK2 + XRCE Agent v2.4.2）与 Jetson 原生安装脚本 | [`docker/Dockerfile`](docker/Dockerfile)、[`scripts/setup_jetson_native.sh`](scripts/setup_jetson_native.sh) |
| 纯 Python 单元测试（不需要 ROS） | [`scripts/run_tests.sh`](scripts/run_tests.sh) |
| 无硬件端到端测试：假 Mid-360 → FAST-LIO2 → 桥接 / 存图 / 分块，对照真值打分 | [`scripts/sim_e2e_test.sh`](scripts/sim_e2e_test.sh) |

```bash
cd ros2_ws && vcs import --recursive src < hummingbird.repos
ros2 launch hummingbird_bringup mapping.launch.py start_xrce_agent:=true xrce_dev:=/dev/ttyTHS1
ros2 launch hummingbird_bringup sim_mapping.launch.py     # 没有雷达和飞控时：假 Mid-360 喂真实链路
```

Ryan 如果有自己一晚上搓出来的版本，欢迎直接推到这个分支或开 PR 覆盖/合并，以他实测过的参数为准。

## 硬件 BOM（设计草案）

下表按团队 2026 年 3 月的项目报告更新过型号；**线序、安装方式和外参仍未用实机核对**。报告里的最终接线图没有 Jetson↔Pixhawk 的数据线，下表的 uXRCE-DDS 链路是本仓库的方案，还没有接。

| 分系统 | 计划型号 / 接口 | 备注 |
| --- | --- | --- |
| 飞控 | Pixhawk 6C（FMU: STM32H743，IO: STM32F103） | PX4；与机载计算机走串口 / Ethernet 的 **uXRCE-DDS** |
| 机载计算机 | NVIDIA Jetson Orin Nano Super 8GB，JetPack 6.2 | 运行 ROS 2 Humble、驱动、FAST-LIO2、任务层 |
| 3D LiDAR | Livox Mid-360 | 典型公开规格：水平 FOV 360°，垂直约 −7°～52°；以 [Livox 官方规格](https://www.livoxtech.com/mid-360) 为准 |
| LiDAR 接口 | Ethernet（100 Mbps 量级）+ 板载 IMU | 经 `livox_ros_driver2` 接入 ROS 2 |
| GNSS / 磁力计 | NEO 4 SE（DroneCAN） | 用于全局锚定与 geofence，不是 LIO 的唯一依赖 |
| 数据链 | Wi-Fi / 网状网（地图与点云），MicroAir LR 900A 数传作低带宽备份；遥控 RadioLink T12D（SBUS） | 见 [docs/air-ground-cooperation.md](docs/air-ground-cooperation.md) |
| 动力与机架 | LeiXun LX450 机架，BrotherHobby Avenger 710KV + 1148 桨，BLHeli 60A 电调，6S 10000 mAh；Tarot TL2996 高压分电板 + PM07 | 取自团队 2026 年 3 月报告，本仓库不定义动力参数 |

## 软件栈

| 层级 | 组件 | 本账号下的 fork / 本仓库路径 |
| --- | --- | --- |
| 飞控 | PX4 Autopilot | [lucascygu/PX4-Autopilot](https://github.com/lucascygu/PX4-Autopilot) ← [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) |
| ROS 2 ↔ PX4 | `px4_msgs`（uXRCE-DDS）；`px4_ros_com` 仅作参考，不装到机上 | [lucascygu/px4_msgs](https://github.com/lucascygu/px4_msgs)、[lucascygu/px4_ros_com](https://github.com/lucascygu/px4_ros_com) |
| Mid-360 驱动与 SDK | `livox_ros_driver2`、Livox-SDK2 | [lucascygu/livox_ros_driver2](https://github.com/lucascygu/livox_ros_driver2)、[lucascygu/Livox-SDK2](https://github.com/lucascygu/Livox-SDK2) |
| LIO / 建图 | FAST-LIO2 的 ROS 2 移植 | [lucascygu/FAST_LIO_ROS2](https://github.com/lucascygu/FAST_LIO_ROS2) ← [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)（fork 网络的 source 是 [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)） |
| 导航 | Nav2 | [lucascygu/navigation2](https://github.com/lucascygu/navigation2) |
| 空地协同接口 | `hummingbird_interfaces` | [`ros2_ws/src/hummingbird_interfaces`](ros2_ws/src/hummingbird_interfaces) |
| 任务层安全门控 | `hummingbird_task_agent` | [`ros2_ws/src/hummingbird_task_agent`](ros2_ws/src/hummingbird_task_agent) |

关于无法再单独 fork `hku-mars/FAST_LIO` 的说明：GitHub 对同一 fork 网络（fork network）每个账号只允许一个 fork。`Ericsii/FAST_LIO_ROS2` 本身就是 `hku-mars/FAST_LIO` 的 fork，因此本账号只保留 [lucascygu/FAST_LIO_ROS2](https://github.com/lucascygu/FAST_LIO_ROS2)。上游原版仍在 [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)（GPL-2.0）。

ROS 2 发行版是 **Humble**（JetPack 6.2 / Ubuntu 22.04，团队报告记录），PX4 固件是 **v1.16.1**，`px4_msgs` 相应钉在 `release/1.16`。

## 文档

- [架构分层](docs/architecture.md)（飞控、感知、建图、导航、任务 / agent 层，含 mermaid 图）
- [空地协同接口草图](docs/air-ground-cooperation.md)（话题 / 服务、地图共享、坐标系对齐、链路选项）
- [任务层 agent 与安全门控](docs/task-agent-design.md)（LLM / agent 只出高层任务；**禁止直接发执行器指令**；geofence；arm / takeoff 需人工确认）

## 本仓库的 ROS 2 骨架

```text
ros2_ws/src/
  hummingbird_interfaces/     # GroundTask、MapTile、HighLevelSetpoint、RequestMission、ApproveMission
  hummingbird_task_agent/     # mission_gate 节点：只发布高层 setpoint 请求；站点 geofence 在 config/mission_gate.yaml
  hummingbird_bringup/        # （ryan-rebuild）Mid-360 + FAST-LIO2 + 桥接 + 存图 + 地图分块的启动与配置，以及假 Mid-360
  hummingbird_px4_bridge/     # （ryan-rebuild）FAST-LIO 里程计 → PX4 外部视觉输入
src/                          # 预留给 Ryan 放入重建代码
```

在已安装 ROS 2 的环境中（已在 x86_64 的 ROS 2 Humble 和 Jazzy 上 `colcon build` 并跑过测试，GitHub Actions 每次推送都会重跑；**没有上过实机，也没有在 Jetson 上编译过**）：

```bash
bash scripts/run_tests.sh                      # 纯 Python 单测，不需要 ROS
cd ros2_ws
colcon build --packages-select hummingbird_interfaces hummingbird_task_agent
source install/setup.bash
ros2 launch hummingbird_task_agent mission_gate.launch.py
colcon test --packages-select hummingbird_task_agent && colcon test-result --verbose
```

`mission_gate` 接受两种入口，走同一套校验：`/hummingbird/mission/request` 服务（批准位随请求一起给），以及 `/hummingbird/ground/task` 话题（先挂起，等操作员调 `/hummingbird/mission/approve` 批准）。返航和降落请求从不被拦。**不代表已在 PX4 或实机上验证**；目前也没有节点把放行的 setpoint 转给 PX4。


## 相关项目：英伟达小鸭子 / Open Duck Mini

用户提到的「英伟达小鸭子」，对应的是 Disney **BDX** 风格开源迷你鸭机器人，以及由 NVIDIA 等发起的 **Newton** 物理引擎（常在 GTC 一类场合与仿真 / 具身项目一起出现）。这些仓库**不是**本无人机飞控栈的一部分，只作为相关具身智能项目 fork 到同一账号下便于查阅。本仓库也没有导入它们的源码。

| 项目 | 作用 | 本账号 fork | 上游 |
| --- | --- | --- | --- |
| Open Duck Mini | 开源迷你 BDX 机器人（硬件 / BOM / 设计中枢，约 42 cm，Apache-2.0） | [lucascygu/Open_Duck_Mini](https://github.com/lucascygu/Open_Duck_Mini) | [apirrone/Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini) |
| Open Duck Mini Runtime | 实机运行时 | [lucascygu/Open_Duck_Mini_Runtime](https://github.com/lucascygu/Open_Duck_Mini_Runtime) | [apirrone/Open_Duck_Mini_Runtime](https://github.com/apirrone/Open_Duck_Mini_Runtime) |
| Open Duck Playground | Open Duck 的 MuJoCo Playground RL 环境 | [lucascygu/Open_Duck_Playground](https://github.com/lucascygu/Open_Duck_Playground) | [apirrone/Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground) |
| Newton | 基于 NVIDIA Warp 的开源 GPU 物理引擎（Linux Foundation 项目，由 Disney Research、Google DeepMind 与 NVIDIA 发起） | [lucascygu/newton](https://github.com/lucascygu/newton) | [newton-physics/newton](https://github.com/newton-physics/newton) |

未把其他同名社区衍生仓库（例如第三方 Viewer、HAT、非官方 Playground 实验）一并 fork；上面四个是上游本体与同作者的运行时 / 训练场，以及 NVIDIA 参与发起的 Newton 引擎。

## 许可

本仓库中**原创**的文档、接口定义与 `hummingbird_task_agent` 骨架以 [MIT License](LICENSE) 发布。

各 fork 与上游项目保留各自的许可证，**不要把它们重新声明为 MIT**：

| 项目 | 许可证（以各仓库 LICENSE 为准） |
| --- | --- |
| [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)、[Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | GPL-2.0 |
| [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)、[Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | 上游 `LICENSE.txt` 写明相应部分为 MIT（附 Livox 说明） |
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | 混合许可：按各包 `package.xml`；未另行标注的文件为 Apache-2.0；另有 LGPL-2.1-or-later、BSD-3-Clause |
| [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)、[PX4/px4_ros_com](https://github.com/PX4/px4_ros_com)、[PX4/px4_msgs](https://github.com/PX4/px4_msgs) | BSD-3-Clause |
| [newton-physics/newton](https://github.com/newton-physics/newton)、[apirrone/Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini) | Apache-2.0（Newton 文档为 CC-BY-4.0） |
| [apirrone/Open_Duck_Mini_Runtime](https://github.com/apirrone/Open_Duck_Mini_Runtime)、[apirrone/Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground) | 上游根目录未见 LICENSE / `pyproject.toml` 也未声明许可证；fork 仅作存档，不改许可、不把它们当成 MIT |

若后续把 GPL-2.0 的 FAST-LIO2 链进同一发行物，发行许可需要按 GPL 义务另行评估，不能只看本仓库的 MIT。

## 白皮书
见 [docs/whitepaper.md](docs/whitepaper.md)。

## 署名 / Credits

- Lucas：发起人
- Ryan：飞行平台与建图
- Grok：文档、设计与代码协助
- Claude：代码审阅、修正与补全（2026-11-11，见 [docs/ryan-rebuild.md](docs/ryan-rebuild.md) 的审阅记录）
