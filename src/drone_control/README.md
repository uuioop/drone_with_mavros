# Drone Control System (无人机控制系统)

这是一个为 PX4 自动驾驶仪设计的、基于 ROS 的高级控制系统。该系统旨在实现模块化的任务控制，集成了视觉导航和 GPS 导航功能，提供稳定可靠的无人机自主控制能力。

## 主要功能

- **任务模式架构**: 系统采用完整的任务模式设计，每个任务都是独立的执行单元，可以轻松切换不同的任务模式。
  - `IdleMission`: 空闲任务模式，作为系统的默认任务状态。
  - `GPSNavMission`: GPS 导航任务模式，负责根据预定义的航点序列执行导航。
  - `ConfirmLicenseMission`: 号牌确认任务模式，负责搜索并确认目标号牌(当前版本未实现)。
  - `PrecisionLandMission`: 精准降落任务模式，利用 ArUco 标记实现高精度的视觉着陆。
  - 其他任务模式正在开发中，将集成GPS导航、精准降落和号牌确认等功能。

- **状态机驱动**: 每个任务内部都使用状态机（FSM）来管理其复杂的逻辑流程，使得任务执行过程清晰、稳定且易于扩展。

- **MAVROS 通信**: 通过 `MavrosBridge` 与飞控进行底层通信，负责模式切换、解锁/上锁以及发送速度/位置指令。

- **硬件抽象**: `StatusMonitor` 类订阅了所有必要的 MAVROS 话题，为上层应用提供了统一、线程安全的状态接口。

- **视觉处理**: 支持 ArUco 标记检测和号牌识别，用于精确导航和任务确认。

## 功能实现状态

- ✅ **精准降落**: 已实现，支持基于 ArUco 标记的高精度视觉着陆
- ✅ **GPS导航**: 已实现，支持基于 GPS 航点的任务执行
- ⏳ **号牌确认**: 等待号牌模型确定后实现

## 使用方法

系统支持两种使用模式：

### 模式一：直接精准降落
直接启动精准降落模式，无人机将在当前位置附近搜索 ArUco 标记并执行精准降落：

```bash
# 在单独终端中启动服务调用
rosservice call /start_flight 0 0 0 0 true
```

### 模式二：GPS导航后精准降落
先执行GPS导航到指定位置，然后自动切换到精准降落模式：

```bash
# 在单独终端中启动服务调用，只需提供前两个参数（经纬度）
rosservice call /start_flight 47.3979711 8.5461636 0 0 false
```

### 参数说明
- 参数1-2: 目标位置的纬度和经度（单位：度）
- 参数3-4: 目标高度和偏航角（当前版本未使用，设为0）
- 参数5: 是否直接精准降落（true=直接降落，false=先GPS导航再降落）

## 系统架构

系统采用分层架构设计：

1. **底层通信层**: 由 `MavrosBridge` 负责与 PX4 飞控通信
2. **状态监控层**: 由 `StatusMonitor` 提供统一的状态访问接口
3. **任务层**: 包含完整的任务模式，如 `IdleMission` 等
4. **状态机层**: 每个任务内部使用状态机管理复杂任务流程
5. **工具层**: 提供 ArUco 标记处理、号牌识别等辅助功能

## 项目依赖

- ROS Noetic
- PX4 自动驾驶仪
- MAVROS 包
- OpenCV（用于视觉处理）
- Doxygen（用于文档生成）

## 项目结构

- `/include`: 包含所有头文件 (`.h`)。
  - `/core`: 核心控制类，如 `DroneControl`, `MavrosBridge`, `StatusMonitor`。
  - `/MissionNode`: 任务节点基类和具体任务实现。
  - `/StateNode`: 状态机节点定义。
  - `/Tag`: ArUco 标记和号牌处理的相关工具类。
  - `/util`: 通用工具类，如数学工具函数和 SQLite 数据库操作。
- `/src`: 包含所有源文件 (`.cpp`)，目录结构与 `include` 对应。
  - `/core`: 核心控制类实现。
  - `/MissionNode`: 任务节点具体实现。
  - `/StateNode`: 状态机节点实现。
  - `/Tag`: 视觉处理工具实现。
  - `main.cpp`: 程序入口点。
- `/launch`: ROS launch 文件，如 `gazebo.launch`。
- `/docs`: Doxygen 生成的文档输出目录，包含 HTML 和 LaTeX 格式的文档。
- `CMakeLists.txt`: 项目构建配置文件
- `package.xml`: ROS 包描述文件
- `Doxyfile`: Doxygen 文档生成配置文件
- `README.md`: 项目说明文档

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