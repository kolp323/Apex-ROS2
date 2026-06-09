# Apex-ROS2

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://releases.ubuntu.com/22.04/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

[🇨🇳 中文文档 (Chinese Version)](#-中文文档-chinese-version) | [🇬🇧 English Version](#-english-version)

---

## 🇨🇳 中文文档 (Chinese Version)

Apex-ROS2 是一个面向智能车比赛场景的 ROS 2 Humble 综合工程工作空间。它集成了底盘控制、Livox MID360 激光雷达、Astra/USB 相机、FAST-LIO 建图、Nav2 导航、YOLOv8 视觉感知、BEV 障碍物处理、语义代价地图、任务管理以及速度仲裁等模块，形成了一个可直接部署的机器人技术栈。

### 📑 目录
- [技术栈](#-技术栈)
- [系统架构](#-系统架构)
- [技术亮点](#-技术亮点)
- [项目结构](#-项目结构)
- [核心功能包](#-核心功能包)
- [核心 Launch 文件](#-核心-launch-文件)
- [环境与构建](#-环境与构建)
- [运行指南](#-运行指南)
- [运行流程](#-运行流程)
- [开发与复现](#-开发与复现)
- [团队分工](#-团队分工)
- [开源协议与版权声明](#-开源协议与版权声明)

### 🛠️ 技术栈
- **运行环境**: Ubuntu 22.04, ROS 2 Humble, Python 3.10, `colcon`
- **机器人平台**: Ackermann 移动底盘 (搭载 STM32 底层控制器)
- **计算平台**: NVIDIA Jetson Orin Nano 级边缘计算设备
- **传感器**: Livox MID360 激光雷达, Astra Pro Plus 深度相机, USB 摄像头, 底盘里程计/IMU
- **定位建图**: FAST-LIO, AMCL/Nav2 定位, 点云转激光扫描数据转换 (PointCloud to LaserScan)
- **导航控制**: Nav2 Humble, Smac Hybrid-A* 规划算法, Regulated Pure Pursuit 局部控制, 语义代价地图层
- **视觉感知**: YOLOv8 目标检测, 基于 Mask 的相机距离检测, 基于 GPU 的 BEV/IPM 障碍物处理
- **控制安全**: `cmd_vel` 滤波机制, 停车线状态机, 速度限制与障碍物分析

### 🏗️ 系统架构
Apex-ROS2 按照分层自动驾驶技术栈进行组织：
1. **硬件与传感器层**:
   - `turn_on_wheeltec_robot` 与底盘控制器通信并发布基础里程计数据。
   - `livox_ros_driver2` 发布 MID360 点云。
   - `ros2_astra_camera-master` 与 `usb_cam-ros2` 提供视觉输入。
   - `wheeltec_robot_urdf` 提供机器人模型与可视化资源。
2. **定位与建图层**:
   - `FAST_LIO_ROS2` 运行激光惯性建图。
   - Nav2 定位与地图服务器组件提供导航所需的地图和坐标系流。
   - 点云转换模块将 3D 点云降维处理为二维 `LaserScan` 供各类避障算法使用。
3. **感知层**:
   - `yolo_detector` 运行基于 YOLOv8 的目标检测，用于识别数字目标和任务提示。
   - `distance_detector` 监控被遮罩的相机区域，用于近场危险区域事件检测。
   - `bev_obstacle_detector` 使用逆透视映射 (IPM) 将前视相机输入转换为鸟瞰图，以便处理障碍物。
   - `costmap_process` 发布可与 Nav2 代价地图结合的语义层与可视化层。
4. **决策与导航层**:
   - Nav2 处理全局规划、局部控制、生命周期管理以及地图服务。
   - 任务管理器 (Mission Manager) 负责对任务目标进行排序，进行目标去抖并协调感知与导航目标。
5. **控制与安全层**:
   - `cmd_vel_tools`、BEV 处理和距离检测在导航节点输出速度后、指令到达底盘前，进行二次滤波与安全判定。
   - 包含正常行驶、受控停车以及保持/冷却等流程的状态机控制。

### ✨ 技术亮点
- **多模态感知**: 激光雷达提供高精度的几何定位和建图能力，而 YOLO 和 BEV 处理则补充了任务语义以及近场障碍物感知。
- **BEV/IPM 障碍物表征**: 利用单应性矩阵将前视相机图像投影至鸟瞰图平面，此方法对地面标志及低矮障碍物具有极强的适应性，且可为较高物体自动留出保守的安全冗余。
- **目标去抖动 (Debouncing)**: 任务目标仅在检测框发生显著偏移时重新下发，避免了由于识别不稳定导致 Nav2 频繁重规划的问题。
- **速度仲裁机制**: 介入 Nav2 输出与底盘执行之间的独立滤波层，具备针对停车线、防撞红区以及其它受限环境的安全关卡能力。
- **两阶段 YOLO 训练流**: 目标检测模型首先通过 SVHN 数据集进行预热 (Warm-up) 训练，再使用机器人实拍场景数据进行微调，显著提升了小尺寸数字目标的识别准确率。

### 📂 项目结构
```text
Apex-ROS2/
├── car_ws/                         # 主机器人工作空间：感知、导航、控制与核心驱动
│   └── src/
│       ├── bev_obstacle_detector/  # BEV/IPM 障碍物检测与多点云融合
│       ├── cmd_vel_tools/          # 速度仲裁、障碍物分析及停车线状态机
│       ├── costmap_process/        # 语义地图与代价地图发布节点
│       ├── distance_detector/      # 基于视觉掩膜的近场防撞检测
│       ├── distance_detector_msg/  # 自定义距离检测消息
│       ├── FAST_LIO_ROS2/          # FAST-LIO 激光惯性建图模块
│       ├── navigation2-humble/     # 本地集成的 Nav2 导航栈
│       ├── red_segment_msg/        # 自定义红区（禁行区）消息
│       ├── robot_kcf/              # 视觉追踪功能包
│       ├── ros2_astra_camera-master/ # Astra 相机 ROS 2 驱动
│       ├── serial_ros2/            # 串口通信依赖包
│       ├── turn_on_wheeltec_robot/ # 底盘驱动、TF 树、EKF 及传感器启动
│       ├── usb_cam-ros2/           # USB 相机 ROS 2 驱动
│       ├── wheeltec_robot_msg/     # 自定义机器人消息
│       ├── wheeltec_robot_urdf/    # 机器人 URDF 模型及 RViz 资源
│       └── yolo_detector/          # YOLO 识别、任务管理及系统集成启动入口
├── ws_livox/                       # Livox 激光雷达独立工作空间
│   └── src/
│       ├── livox_ros_driver2/      # Livox ROS 2 驱动
│       └── pointcloud_to_laserscan-humble/ # 点云转 LaserScan
├── install_components.sh           # Ubuntu/ROS 依赖快速安装脚本
└── README.md
```

### 📦 核心功能包
| 功能包 | 所在工作区 | 作用 |
| --- | --- | --- |
| `yolo_detector` | `car_ws` | YOLOv8 视觉检测、任务管理、状态机决策与系统集成启动入口 |
| `turn_on_wheeltec_robot` | `car_ws` | 底盘驱动、串口通信、TF 坐标树、EKF 滤波及传感器启动文件 |
| `FAST_LIO_ROS2` | `car_ws` | 基于 FAST-LIO 的激光惯性里程计建图与定位 |
| `navigation2-humble` | `car_ws` | 提供地图服务器、规划控制及生命周期管理的 Nav2 导航栈 |
| `livox_ros_driver2` | `ws_livox` | Livox 激光雷达 (含 MID360) 的 ROS 2 驱动与配置文件 |
| `pointcloud_to_laserscan-humble`| `ws_livox` | 3D 点云到 2D 激光雷达扫描线的转换 |
| `bev_obstacle_detector` | `car_ws` | 鸟瞰图 (BEV/IPM) 障碍物检测，多点云融合与速度障碍分析 |
| `distance_detector` | `car_ws` | 基于相机掩膜 (Mask) 的距离与危险区域检测 |
| `costmap_process` | `car_ws` | 语义地图处理、代价地图发布及可视化 |
| `cmd_vel_tools` | `car_ws` | 速度滤波、障碍物分析判断与防撞红区处理逻辑 |

### 🚀 核心 Launch 文件
| Launch 文件 | 用途 |
| --- | --- |
| `car_ws/src/yolo_detector/launch/main_nav.launch.py` | 核心集成启动文件。按顺序启动底盘、静态 TF、FAST-LIO、YOLO、距离检测、语义/代价地图节点、地图服务器、Nav2、Livox 及 Astra 相机。 |
| `car_ws/src/yolo_detector/launch/multi_robot_startup.launch.py` | 面向导航的启动流，包含底盘、TF、FAST-LIO、地图服务器、Nav2 及 Livox。 |
| `car_ws/src/yolo_detector/launch/yolo_detect.launch.py` | 启动 YOLO 目标检测节点，使用 `config/yolo_params.yaml`。 |
| `car_ws/src/yolo_detector/launch/mission_manager.launch.py` | 启动任务管理器，使用 `config/mission_manager_params.yaml`。 |
| `car_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py` | 启动 `fastlio_mapping`，默认加载 `config/mid360.yaml`。 |
| `ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_map_launch.py` | 启动 Livox 驱动及点云转换节点（适配 MID360 制图需求）。 |

### ⚙️ 环境与构建

#### 推荐的基础环境
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `colcon`, `rosdep`, CMake 及 GNU 编译工具

#### 1. 安装 ROS 依赖
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip git cmake build-essential
sudo rosdep init || true
rosdep update
bash install_components.sh
```

#### 2. 安装 Livox-SDK2
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
sudo ldconfig
```

#### 3. 编译 Livox 工作空间
```bash
source /opt/ros/humble/setup.bash
cd ws_livox/src/livox_ros_driver2
./build.sh humble
cd ../../..
source install/setup.bash
```

#### 4. 编译主机器人工作空间
```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
cd car_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
cd ..
```

### 🏃 运行指南

**新终端需 Source 环境变量：**
```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
source car_ws/install/setup.bash
```

**综合导航启动：**
```bash
ros2 launch yolo_detector main_nav.launch.py \
  use_rviz:=true \
  launch_camera:=true \
  launch_lidar:=true \
  yolo_vis:=true \
  distance_vis:=true \
  sensor_startup_delay:=10.0
```
> 你可以通过传递 `map_yaml_file:=/绝对路径/map.yaml` 来切换地图。

**单独模块启动测试：**
```bash
# 启动 Livox
ros2 launch livox_ros_driver2 msg_MID360_map_launch.py
# 启动 FAST-LIO
ros2 launch fast_lio mapping.launch.py
# 启动 YOLO 识别
ros2 launch yolo_detector yolo_detect.launch.py enable_vis:=true
```

### 🔄 运行流程
1. 启动 `turn_on_wheeltec_robot` 底盘控制节点。
2. 发布 `map` 与 `camera_init` 之间的静态坐标系。
3. 启动 FAST-LIO 模块提供里程计与建图。
4. 激活 Nav2 地图服务器与导航栈。
5. 在预设延时后，安全启动激光雷达及相机传感器节点。
6. 启动视觉检测及代价地图相关处理管线。
7. 任务管理器与 Nav2 协同，规划出合适的运动目标。
8. 经过速度仲裁机制滤波后，下发控制指令到底盘。

### 💻 开发与复现
- 本综合工程强依赖对应硬件配置。若需要在非目标机器人上进行逻辑测试，请注意禁用需要依赖硬件的 Launch 文件。
- BEV/IPM 识别依赖平坦地面假设，如遇坡道、剧烈俯仰震荡、强光环境等可能会降低系统可靠性。
- 部分 Launch 文件包含了固定的绝对路径（如 RViz 及默认地图路径），实际部署时请通过 Launch 参数或修改源码进行适配调整。

### 👥 团队分工
本项目由团队成员协同开发，核心模块的分工如下：

| 团队成员 | 核心负责方向 | 关联功能包 | 工作亮点 |
| :--- | :--- | :--- | :--- |
| **朱首赫** | 视觉检测、状态决策与 BEV 感知 | `yolo_detector`, `bev_obstacle_detector`, 任务决策流 | 实现数字目标识别与状态机决策，开发基于鸟瞰图 (BEV/IPM) 的底层避障策略，优化了整体感知数据管线。 |
| **冯曦熠** | 导航算法与控制滤波 | `navigation2-humble`, `cmd_vel_tools` | 负责 Nav2 导航栈规划器的调参，设计并实现了速度控制仲裁机制。 |
| **许震宇** | 模型训练与代价地图 | `yolo_detector` (训练侧), `costmap_process` | 负责 YOLO 目标检测模型的训练，开发了代价地图修改机制。 |

### 📜 开源协议与版权声明
- **代码许可**: 本项目的源代码采用 [MIT License](LICENSE) 开源协议。欢迎各位开发者基于此项目进行学习、修改与二次开发。
- **文档许可与包装声明**: 本项目的**整体工程化包装、系统架构文档撰写及 README 维护**均由 **朱首赫** 独立完成。
- 我们非常欢迎大家参考和借鉴本项目的架构设计与代码结构。但在开源分享的同时，也恳请各位开发者尊重原创作者的劳动成果与学术诚信：**请勿在未经授权的情况下，直接将本仓库的整体结构、文档文案或展示排版“原样照搬”。** 如果本项目在工程规范或文档架构上对您有所启发，在合理引用的同时注明出处，将是对开源贡献者最大的鼓励与支持。

---
<br>

## 🇬🇧 English Version

Apex-ROS2 is a ROS 2 Humble workspace for an autonomous Ackermann robot used in intelligent vehicle competition scenarios. It combines chassis control, Livox MID360 LiDAR, Astra/USB cameras, FAST-LIO mapping, Nav2 navigation, YOLOv8 perception, BEV obstacle processing, semantic costmap updates, mission management, and velocity arbitration into one deployable robot stack.

### 📑 Table of Contents
- [Technical Stack](#-technical-stack-1)
- [System Architecture](#-system-architecture-1)
- [Method Highlights](#-method-highlights-1)
- [Repository Structure](#-repository-structure-1)
- [Main Packages](#-main-packages-1)
- [Key Launch Files](#-key-launch-files-1)
- [Environment and Build](#-environment-and-build-1)
- [Run](#-run-1)
- [Runtime Flow](#-runtime-flow-1)
- [Reproduction Notes](#-reproduction-notes-1)
- [Development Workflow](#-development-workflow-1)
- [Team Contributions](#-team-contributions-1)
- [License & Authorship Statement](#-license--authorship-statement-1)

### 🛠️ Technical Stack
- **Runtime**: Ubuntu 22.04, ROS 2 Humble, Python 3.10, `colcon`
- **Robot platform**: Ackermann mobile chassis with an STM32-based low-level controller
- **Compute**: NVIDIA Jetson Orin Nano class edge computer
- **Sensors**: Livox MID360 LiDAR, Astra Pro Plus depth camera, USB camera, odometry/IMU from the chassis stack
- **Localization and mapping**: FAST-LIO, AMCL/Nav2 localization, point cloud to laser scan conversion
- **Navigation**: Nav2 Humble, Smac Hybrid-A* style planning, Regulated Pure Pursuit local control, costmap layers
- **Perception**: YOLOv8 target detection, camera mask distance detection, GPU-oriented BEV/IPM obstacle processing
- **Control safety**: `cmd_vel` filtering, stop-line state machine, obstacle analysis, velocity limiting

### 🏗️ System Architecture
Apex-ROS2 is organized as a layered autonomous driving stack.

1. **Hardware and sensor layer**
   - `turn_on_wheeltec_robot` communicates with the chassis controller and publishes base odometry data.
   - `livox_ros_driver2` publishes MID360 point clouds.
   - `ros2_astra_camera-master` and `usb_cam-ros2` provide visual input.
   - `wheeltec_robot_urdf` provides the robot model and visualization assets.

2. **Localization and mapping layer**
   - `FAST_LIO_ROS2` runs LiDAR-inertial mapping with `config/mid360.yaml` as the default MID360 configuration.
   - Nav2 localization and map server components provide the map and frame flow required by navigation.
   - `pointcloud_to_laserscan-humble` converts point clouds to scan messages for modules that expect `LaserScan` input.

3. **Perception layer**
   - `yolo_detector` runs YOLOv8-based target detection for numbered targets and task cues.
   - `distance_detector` monitors a masked camera region for near-field danger-zone events.
   - `bev_obstacle_detector` uses inverse perspective mapping (IPM) to convert front-view camera input into a bird's-eye representation for obstacle processing.
   - `costmap_process` publishes semantic and visualization layers that can be combined with Nav2 costmaps.

4. **Decision and navigation layer**
   - Nav2 handles global planning, local control, lifecycle management, and map serving.
   - `mission_manager` sequences task goals, republishes goals only when target positions change significantly, and coordinates perception outputs with navigation targets.
   - The competition flow can send ordered goals such as bonus-point and finish-point targets instead of repeatedly navigating to one unstable detection.

5. **Control and safety layer**
   - Nav2 produces `cmd_vel_nav2`.
   - `cmd_vel_tools`, BEV processing, and distance detection filter or analyze velocity commands before they reach the chassis.
   - Stop-line handling uses a simple state machine: clear driving, controlled stopping, and holding/cooldown to avoid repeated triggers.

### ✨ Method Highlights
- **Multi-modal perception**: LiDAR supports localization and geometry, while YOLO and BEV processing provide task semantics and near-field obstacle awareness.
- **BEV/IPM obstacle representation**: The camera view is projected into a bird's-eye plane using a homography calibration. This is practical for ground markers and low obstacles, and the stretching effect of taller objects can increase conservative obstacle margins.
- **Goal debouncing**: Mission targets are only republished when the detected goal moves beyond a threshold, reducing unnecessary Nav2 replanning caused by unstable detection boxes.
- **Velocity arbitration**: A filtering layer between Nav2 output and chassis command acts as a safety gate for stop lines, red segments, and obstacle-related constraints.
- **Two-stage YOLO training workflow**: The detection model was trained with an SVHN warm-up stage followed by real robot-scene data, improving deployment accuracy for small numbered targets.

### 📂 Repository Structure
```text
Apex-ROS2/
├── car_ws/                         # Main robot, perception, navigation and control workspace
│   └── src/
│       ├── bev_obstacle_detector/  # BEV/IPM obstacle detection and point cloud fusion
│       ├── cmd_vel_tools/          # Velocity arbitration, obstacle analysis and stop-line utilities
│       ├── costmap_process/        # Semantic map and costmap publishing nodes
│       ├── distance_detector/      # Mask-based visual proximity detection
│       ├── distance_detector_msg/  # Custom distance detection messages
│       ├── FAST_LIO_ROS2/          # FAST-LIO LiDAR-inertial mapping integration
│       ├── navigation2-humble/     # Vendored Nav2 Humble stack used by this workspace
│       ├── red_segment_msg/        # Custom red segment messages
│       ├── robot_kcf/              # Visual tracking package
│       ├── ros2_astra_camera-master/ # Astra camera ROS 2 driver
│       ├── serial_ros2/            # Serial communication dependency package
│       ├── turn_on_wheeltec_robot/ # Chassis driver, TF, EKF and sensor launch files
│       ├── usb_cam-ros2/           # USB camera ROS 2 driver
│       ├── wheeltec_robot_msg/     # Custom robot messages
│       ├── wheeltec_robot_urdf/    # Robot URDF, meshes and RViz resources
│       └── yolo_detector/          # YOLO detection, mission manager and system launch entrypoints
├── ws_livox/                       # Livox LiDAR workspace
│   └── src/
│       ├── livox_ros_driver2/      # Livox ROS 2 driver and MID360 launch/config files
│       └── pointcloud_to_laserscan-humble/
├── install_components.sh           # Ubuntu/ROS dependency installation helper
└── README.md
```

### 📦 Main Packages
| Package | Workspace | Role |
| --- | --- | --- |
| `yolo_detector` | `car_ws` | YOLOv8 perception, mission management, decision launch files and full-system startup entrypoints |
| `turn_on_wheeltec_robot` | `car_ws` | Chassis driver, serial communication, TF, EKF and sensor startup launch files |
| `FAST_LIO_ROS2` / `fast_lio` | `car_ws` | LiDAR-inertial mapping and localization based on FAST-LIO |
| `navigation2-humble` | `car_ws` | Nav2 stack for map server, planning, control and lifecycle management |
| `livox_ros_driver2` | `ws_livox` | ROS 2 driver and launch/config files for Livox LiDARs, including MID360 |
| `pointcloud_to_laserscan-humble` | `ws_livox` | PointCloud2 to LaserScan conversion |
| `bev_obstacle_detector` | `car_ws` | BEV/IPM obstacle detection, point cloud merging and velocity-related obstacle processing |
| `distance_detector` | `car_ws` | Camera-mask-based distance and danger-zone detection |
| `costmap_process` | `car_ws` | Semantic map processing, costmap publishing and visualization |
| `cmd_vel_tools` | `car_ws` | Velocity limiting, obstacle analysis, debug node and red segment processing |

### 🚀 Key Launch Files
| Launch file | Purpose |
| --- | --- |
| `car_ws/src/yolo_detector/launch/main_nav.launch.py` | Main integrated startup file. Starts chassis, static TF, FAST-LIO, YOLO, distance detection, semantic/costmap nodes, map server, Nav2, Livox and Astra camera with lifecycle sequencing. |
| `car_ws/src/yolo_detector/launch/multi_robot_startup.launch.py` | Navigation-oriented startup flow for chassis, TF, FAST-LIO, map server, Nav2 and Livox. |
| `car_ws/src/yolo_detector/launch/yolo_detect.launch.py` | Starts the YOLO detection node with `config/yolo_params.yaml`. |
| `car_ws/src/yolo_detector/launch/mission_manager.launch.py` | Starts the mission manager with `config/mission_manager_params.yaml`. |
| `car_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py` | Starts `fastlio_mapping`, defaulting to `config/mid360.yaml`. |
| `ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_map_launch.py` | Starts the Livox driver and pointcloud-to-laserscan bridge for MID360 map usage. |

### ⚙️ Environment and Build
Recommended base environment:
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `colcon`, `rosdep`, CMake and GNU build tools
- Nav2, image transport, PCL, robot localization, BehaviorTree.CPP and OMPL dependencies
- Livox-SDK2 and Livox ROS Driver 2 for MID360 LiDAR input
- Astra and USB camera driver dependencies
- Python perception dependencies such as OpenCV, NumPy, PyTorch and YOLO-related packages

#### 1. Install ROS dependencies
After installing ROS 2 Humble, install the package dependencies used by the workspace:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip git cmake build-essential
sudo rosdep init || true
rosdep update
bash install_components.sh
```

#### 2. Install Livox-SDK2
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
sudo ldconfig
```

#### 3. Build the Livox workspace
```bash
source /opt/ros/humble/setup.bash
cd ws_livox/src/livox_ros_driver2
./build.sh humble
cd ../../..
source install/setup.bash
```

#### 4. Build the main robot workspace
```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
cd car_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
cd ..
```

### 🏃 Run

**Sourcing environments in a new terminal:**
```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
source car_ws/install/setup.bash
```

**Integrated autonomous navigation stack:**
```bash
ros2 launch yolo_detector main_nav.launch.py \
  use_rviz:=true \
  launch_camera:=true \
  launch_lidar:=true \
  yolo_vis:=true \
  distance_vis:=true \
  sensor_startup_delay:=10.0
```
> Change map by appending `map_yaml_file:=/absolute/path/to/map.yaml`

**Individual modules:**
```bash
# Livox driver
ros2 launch livox_ros_driver2 msg_MID360_map_launch.py
# FAST-LIO mapping
ros2 launch fast_lio mapping.launch.py
# YOLO detection
ros2 launch yolo_detector yolo_detect.launch.py enable_vis:=true
```

### 🔄 Runtime Flow
1. Start the robot chassis driver through `turn_on_wheeltec_robot`.
2. Publish the static transform between `map` and `camera_init`.
3. Start FAST-LIO for localization or mapping.
4. Start the Nav2 map server and activate it through lifecycle transitions.
5. Start Nav2 navigation after the map server becomes active.
6. Start Livox LiDAR and camera nodes after the configured sensor delay.
7. Run YOLO detection, distance detection and costmap processing.
8. Let mission management and Nav2 coordinate target selection and motion.
9. Apply velocity filtering and obstacle analysis before sending motion commands to the chassis.

### 💻 Reproduction Notes
- The integrated system is hardware-dependent. For desktop inspection, build the workspaces first and launch individual modules before running `main_nav.launch.py`.
- Check serial device names, camera calibration, LiDAR configuration, TF frames, and map paths on the target robot before running the full stack.
- The BEV/IPM pipeline assumes a mostly planar ground surface. Slopes, camera pitch/roll vibration, strong lighting changes, and reflective surfaces can reduce reliability.

### 👥 Team Contributions
| Team Member | Core Responsibilities | Main Packages | Work Highlights |
| :--- | :--- | :--- | :--- |
| **Zhu Shouhe** | Vision Detection, Decision Making, BEV Perception | `yolo_detector`, `bev_obstacle_detector`, Mission Manager | Implemented digit target recognition and state machine decision logic. Developed a low-level obstacle avoidance strategy based on Bird's-Eye View (BEV/IPM) and optimized the overall perception data pipeline. |
| **Feng Xiyi** | Navigation Algorithms, Velocity Control | `navigation2-humble`, `cmd_vel_tools` | Responsible for parameter tuning of the Nav2 navigation stack planners. Designed and implemented the velocity control arbitration mechanism. |
| **Xu Zhenyu** | Model Training, Costmap Modification | `yolo_detector` (Training), `costmap_process` | Responsible for training the YOLO target detection model. Developed the costmap modification mechanisms. |

### 📜 License & Authorship Statement
- **Code License**: The source code of this project is released under the [MIT License](LICENSE). You are welcome to use, modify, and distribute the code for learning and development.
- **Documentation Authorship**: The **overall engineering architecture, project packaging, and documentation (including this README)** were independently designed and written by **Zhu Shouhe**. 
- We strongly encourage learning from our architectural design and code structure. However, out of respect for academic integrity and the original author's effort, we kindly request that developers **do not directly copy the repository's structural layout, documentation wording, or project presentation**. If our repository serves as a reference for your project's architecture or documentation, a proper citation or acknowledgment is highly appreciated.
