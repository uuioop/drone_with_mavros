# Drone Control System (无人机控制系统)

这是一个为 PX4 自动驾驶仪设计的、基于 ROS 的高级控制系统。该系统旨在实现模块化的任务控制，集成了视觉导航和 GPS 导航功能，提供稳定可靠的无人机自主控制能力。

## 主要功能

- **多控制器架构**: 系统被设计为可以轻松切换不同的高级控制器，以执行不同的任务。
  - `MissionNavController`: 执行基于 GPS 航点的任务。
  - `ConfirmLicenseController`: 使用视觉识别技术搜索并确认特定号牌。
  - `PrecisionLandController`: 利用 ArUco 标记实现高精度的视觉着陆。

- **状态机驱动**: 每个控制器内部都使用状态机（FSM）来管理其复杂的逻辑流程，使得任务执行过程清晰、稳定且易于扩展。

- **MAVROS 通信**: 通过 `MavrosBridge` 与飞控进行底层通信，负责模式切换、解锁/上锁以及发送速度/位置指令。

- **硬件抽象**: `StatusMonitor` 类订阅了所有必要的 MAVROS 话题，为上层应用提供了统一、线程安全的状态接口。

- **视觉处理**: 支持 ArUco 标记检测和号牌识别，用于精确导航和任务确认。

## 系统架构

系统采用分层架构设计：

1. **底层通信层**: 由 `MavrosBridge` 负责与 PX4 飞控通信
2. **状态监控层**: 由 `StatusMonitor` 提供统一的状态访问接口
3. **控制器层**: 包含多个高级控制器，如 `MissionNavController`、`ConfirmLicenseController` 等
4. **状态机层**: 每个控制器内部使用状态机管理复杂任务流程
5. **工具层**: 提供 ArUco 标记处理、号牌识别等辅助功能

## 项目依赖

- ROS Noetic
- PX4 自动驾驶仪
- MAVROS 包
- OpenCV（用于视觉处理）
- Doxygen（用于文档生成）
- SQLite（用于数据存储,还未使用）

## 项目结构

- `/include`: 包含所有头文件 (`.h`)。
  - `/core`: 核心控制类，如 `DroneControl`, `MavrosBridge`, `StatusMonitor`。
  - `/fsm`: 状态机 (`StateMachine`) 和状态节点 (`StateNode`) 的基类。
  - `/VisualNav`: 视觉导航相关的控制器和状态定义。
  - `/GPSNav`: GPS 导航相关的控制器定义。
  - `/Tag`: ArUco 标记和号牌处理的相关工具类。
  - `/util`: 通用工具类，如数学工具函数和 SQLite 数据库操作。
- `/src`: 包含所有源文件 (`.cpp`)，目录结构与 `include` 对应。
- `/docs`: Doxygen 生成的文档输出目录，包含 HTML 和 LaTeX 格式的文档。
- `CMakeLists.txt`: 项目构建配置文件
- `package.xml`: ROS 包描述文件
- `Doxyfile`: Doxygen 文档生成配置文件

## 文档生成

使用 Doxygen 生成项目文档：

```bash
cd /home/colu/catkin_ws/src/drone_control
doxygen Doxyfile
```

生成的文档将位于 `/docs/html` 目录中。

## 许可证

[在此添加许可证信息]

## 贡献指南

欢迎提交问题报告和功能请求。如需贡献代码，请提交 Pull Request。

## 联系方式

[在此添加项目维护者的联系方式]