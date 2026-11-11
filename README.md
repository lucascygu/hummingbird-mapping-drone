# 蜂鸟 Hummingbird Mapping Drone

面向 **具身智能空中节点**（embodied-intelligence aerial node）的三维建图无人机项目，命名取自「蜂鸟 / Hummingbird」，也与「猫大虫」相关设想一脉相承。

目标硬件与软件栈：

- 飞控：Holybro **Pixhawk 6C** + **PX4**
- 机载计算机：**NVIDIA Jetson**（具体模块型号待确认）
- 激光雷达：**Livox Mid-360**
- 中间件：**ROS 2**
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

目前仓库里还没有 `ryan-rebuild` 分支（将在首次推送时创建）。`main` 上的文档和 `hummingbird_*` 包可以并存，不必互相覆盖。

## 硬件 BOM（设计草案）

下表是按公开规格整理的计划清单，**具体采购型号、线序和安装方式尚未用实机核对**。

| 分系统 | 计划型号 / 接口 | 备注 |
| --- | --- | --- |
| 飞控 | Pixhawk 6C（FMU: STM32H743，IO: STM32F103） | PX4；与机载计算机走串口 / Ethernet 的 **uXRCE-DDS** |
| 机载计算机 | NVIDIA Jetson（Orin NX / Orin Nano 等，待确认） | 运行 ROS 2、驱动、FAST-LIO2、任务层 |
| 3D LiDAR | Livox Mid-360 | 典型公开规格：水平 FOV 360°，垂直约 −7°～52°；以 [Livox 官方规格](https://www.livoxtech.com/mid-360) 为准 |
| LiDAR 接口 | Ethernet（100 Mbps 量级）+ 板载 IMU | 经 `livox_ros_driver2` 接入 ROS 2 |
| GNSS / 磁力计 | Pixhawk 6C 配套 GPS 模块（具体型号待确认） | 用于全局锚定与 geofence，不是 LIO 的唯一依赖 |
| 数据链 | Wi-Fi / 网状网（地图与点云），数传电台作低带宽备份 | 见 [docs/air-ground-cooperation.md](docs/air-ground-cooperation.md) |
| 动力与机架 | 待确认 | 本仓库暂不定义 |

## 软件栈

| 层级 | 组件 | 本账号下的 fork / 本仓库路径 |
| --- | --- | --- |
| 飞控 | PX4 Autopilot | [lucascygu/PX4-Autopilot](https://github.com/lucascygu/PX4-Autopilot) ← [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) |
| ROS 2 ↔ PX4 | `px4_ros_com`、`px4_msgs`（uXRCE-DDS） | [lucascygu/px4_ros_com](https://github.com/lucascygu/px4_ros_com)、[lucascygu/px4_msgs](https://github.com/lucascygu/px4_msgs) |
| Mid-360 驱动与 SDK | `livox_ros_driver2`、Livox-SDK2 | [lucascygu/livox_ros_driver2](https://github.com/lucascygu/livox_ros_driver2)、[lucascygu/Livox-SDK2](https://github.com/lucascygu/Livox-SDK2) |
| LIO / 建图 | FAST-LIO2 的 ROS 2 移植 | [lucascygu/FAST_LIO_ROS2](https://github.com/lucascygu/FAST_LIO_ROS2) ← [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)（fork 网络的 source 是 [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)） |
| 导航 | Nav2 | [lucascygu/navigation2](https://github.com/lucascygu/navigation2) |
| 空地协同接口 | `hummingbird_interfaces` | [`ros2_ws/src/hummingbird_interfaces`](ros2_ws/src/hummingbird_interfaces) |
| 任务层安全门控 | `hummingbird_task_agent` | [`ros2_ws/src/hummingbird_task_agent`](ros2_ws/src/hummingbird_task_agent) |

关于无法再单独 fork `hku-mars/FAST_LIO` 的说明：GitHub 对同一 fork 网络（fork network）每个账号只允许一个 fork。`Ericsii/FAST_LIO_ROS2` 本身就是 `hku-mars/FAST_LIO` 的 fork，因此本账号只保留 [lucascygu/FAST_LIO_ROS2](https://github.com/lucascygu/FAST_LIO_ROS2)。上游原版仍在 [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)（GPL-2.0）。

规划中的 ROS 2 发行版（Humble / Jazzy 等）**尚未选定**，等 Ryan 的重建工作空间或 Jetson 镜像确定后再对齐。

## 文档

- [架构分层](docs/architecture.md)（飞控、感知、建图、导航、任务 / agent 层，含 mermaid 图）
- [空地协同接口草图](docs/air-ground-cooperation.md)（话题 / 服务、地图共享、坐标系对齐、链路选项）
- [任务层 agent 与安全门控](docs/task-agent-design.md)（LLM / agent 只出高层任务；**禁止直接发执行器指令**；geofence；arm / takeoff 需人工确认）

## 本仓库的 ROS 2 骨架

```text
ros2_ws/src/
  hummingbird_interfaces/     # GroundTask、MapTile、HighLevelSetpoint、RequestMission
  hummingbird_task_agent/     # mission_gate 节点：只发布高层 setpoint 请求
src/                          # 预留给 Ryan 放入重建代码
```

在已安装 ROS 2 的环境中（本仓库的 CI / 作者环境**未**跑过 `colcon build` 或实机）：

```bash
cd ros2_ws
colcon build --packages-select hummingbird_interfaces hummingbird_task_agent
source install/setup.bash
ros2 launch hummingbird_task_agent mission_gate.launch.py
```

`mission_gate` 的纯逻辑可用 `python3 -m py_compile` 做语法检查；**不代表已在 PX4 或实机上验证**。


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
