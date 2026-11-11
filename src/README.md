# `src/`：Ryan 重建代码放置处

本文档由 Grok 撰写/协助

Ryan 的原工程代码已全部丢失。本目录是给重建后的工作空间预留的位置，当前只有这份说明，没有从原项目导入的源码。

## 建议放这里的内容

- PX4 / Pixhawk 6C 配置、启动与参数
- Jetson 上的 ROS 2 工作空间摘录（驱动、FAST-LIO2 启动文件、标定）
- Livox Mid-360 外参、IMU 与飞控的标定结果
- 实机网络、uXRCE-DDS 与电源相关笔记

请在本文件补充实际目录结构。不要随意覆盖 `../ros2_ws/src/hummingbird_interfaces` 与 `hummingbird_task_agent`（那是本仓库新建的空地协同 / 门控骨架），除非你有意把两边合并。

也可以把完整重建历史推送到 `ryan-rebuild` 分支，做法见仓库根目录 README 的「如何提交重建代码」。
