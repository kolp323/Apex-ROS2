# Apex-ROS2

[🇨🇳 中文版 (Chinese Version)](#-中文文档-chinese-version)

Apex-ROS2 is a ROS 2 Humble workspace for an autonomous Ackermann robot used in intelligent vehicle competition scenarios. It combines chassis control, Livox MID360 LiDAR, Astra/USB cameras, FAST-LIO mapping, Nav2 navigation, YOLOv8 perception, BEV obstacle processing, semantic costmap updates, mission management, and velocity arbitration into one deployable robot stack.

## 👥 Team Contributions

The Apex-ROS2 stack was collaboratively developed by our team, with core modules distributed as follows:

| Team Member | Core Responsibilities | Main Packages | Work Highlights |
| :--- | :--- | :--- | :--- |
| **Zhu Shouhe** | Vision Detection, Decision Making, BEV Perception | `yolo_detector`, `bev_obstacle_detector`, Mission Manager | Implemented digit target recognition and state machine decision logic. Developed a low-level obstacle avoidance strategy based on Bird's-Eye View (BEV/IPM) and optimized the overall perception data pipeline. |
| **Feng Xiyi** | Navigation Algorithms, Velocity Control | `navigation2-humble`, `cmd_vel_tools` | Responsible for parameter tuning of the Nav2 navigation stack planners. Designed and implemented the velocity control arbitration mechanism. |
| **Xu Zhenyu** | Model Training, Costmap Modification | `yolo_detector` (Training), `costmap_process` | Responsible for training the YOLO target detection model. Developed the costmap modification mechanisms. |

## 📜 License & Authorship Statement

### Code License
The source code of this project is released under the [MIT License](LICENSE). You are welcome to use, modify, and distribute the code for learning and development.

### Documentation Authorship
The **overall engineering architecture, project packaging, and documentation (including this README)** were independently designed and written by **Zhu Shouhe**. 

We strongly encourage learning from our architectural design and code structure. However, out of respect for academic integrity and the original author's effort, we kindly request that developers **do not directly copy the repository's structural layout, documentation wording, or project presentation as their own independent work**. If our repository serves as a reference for your project's architecture or documentation, a proper citation or acknowledgment is highly appreciated.

---

## Technical Stack

- **Runtime**: Ubuntu 22.04, ROS 2 Humble, Python 3.10, `colcon`
- **Robot platform**: Ackermann mobile chassis with an STM32-based low-level controller
- **Compute**: NVIDIA Jetson Orin Nano class edge computer
- **Sensors**: Livox MID360 LiDAR, Astra Pro Plus depth camera, USB camera, odometry/IMU from the chassis stack
- **Localization and mapping**: FAST-LIO, AMCL/Nav2 localization, point cloud to laser scan conversion
- **Navigation**: Nav2 Humble, Smac Hybrid-A* style planning, Regulated Pure Pursuit local control, costmap layers
- **Perception**: YOLOv8 target detection, camera mask distance detection, GPU-oriented BEV/IPM obstacle processing
- **Control safety**: `cmd_vel` filtering, stop-line state machine, obstacle analysis, velocity limiting

## Repository Structure

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

## System Architecture

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

## Method Highlights

- **Multi-modal perception**: LiDAR supports localization and geometry, while YOLO and BEV processing provide task semantics and near-field obstacle awareness.
- **BEV/IPM obstacle representation**: The camera view is projected into a bird's-eye plane using a homography calibration. This is practical for ground markers and low obstacles, and the stretching effect of taller objects can increase conservative obstacle margins.
- **Goal debouncing**: Mission targets are only republished when the detected goal moves beyond a threshold, reducing unnecessary Nav2 replanning caused by unstable detection boxes.
- **Velocity arbitration**: A filtering layer between Nav2 output and chassis command acts as a safety gate for stop lines, red segments, and obstacle-related constraints.
- **Two-stage YOLO training workflow**: The detection model was trained with an SVHN warm-up stage followed by real robot-scene data, improving deployment accuracy for small numbered targets.

## Main Packages

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
| `wheeltec_robot_msg` | `car_ws` | Custom robot message definitions |
| `distance_detector_msg` | `car_ws` | Custom distance detection message definitions |
| `red_segment_msg` | `car_ws` | Custom red segment message definitions |
| `wheeltec_robot_urdf` | `car_ws` | Robot model, meshes and RViz resources |
| `robot_kcf` | `car_ws` | Visual tracking package |

## Key Launch Files

| Launch file | Purpose |
| --- | --- |
| `car_ws/src/yolo_detector/launch/main_nav.launch.py` | Main integrated startup file. Starts chassis, static TF, FAST-LIO, YOLO, distance detection, semantic/costmap nodes, map server, Nav2, Livox and Astra camera with lifecycle sequencing. |
| `car_ws/src/yolo_detector/launch/multi_robot_startup.launch.py` | Navigation-oriented startup flow for chassis, TF, FAST-LIO, map server, Nav2 and Livox. |
| `car_ws/src/yolo_detector/launch/yolo_detect.launch.py` | Starts the YOLO detection node with `config/yolo_params.yaml`. |
| `car_ws/src/yolo_detector/launch/mission_manager.launch.py` | Starts the mission manager with `config/mission_manager_params.yaml`. |
| `car_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py` | Starts `fastlio_mapping`, defaulting to `config/mid360.yaml`. |
| `ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_map_launch.py` | Starts the Livox driver and pointcloud-to-laserscan bridge for MID360 map usage. |
| `car_ws/src/turn_on_wheeltec_robot/launch/base_serial.launch.py` | Starts the robot base serial driver. |
| `car_ws/src/turn_on_wheeltec_robot/launch/wheeltec_camera.launch.py` | Starts camera-related robot components. |
| `car_ws/src/bev_obstacle_detector/launch/bev_detector.launch.py` | Starts BEV obstacle detection. |
| `car_ws/src/costmap_process/launch/map_vis.launch.py` | Starts costmap/map visualization nodes. |

## Environment and Build

Recommended base environment:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `colcon`, `rosdep`, CMake and GNU build tools
- Nav2, image transport, PCL, robot localization, BehaviorTree.CPP and OMPL dependencies
- Livox-SDK2 and Livox ROS Driver 2 for MID360 LiDAR input
- Astra and USB camera driver dependencies
- Python perception dependencies such as OpenCV, NumPy, PyTorch and YOLO-related packages

Hardware expected by the integrated launch files:

- Ackermann mobile robot chassis
- STM32-based chassis controller
- Livox MID360 LiDAR
- Astra camera and/or USB camera
- Robot serial controller, defaulting to `/dev/wheeltec_controller` in `base_serial.launch.py`

### 1. Install ROS dependencies

After installing ROS 2 Humble, install the package dependencies used by the workspace:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip git cmake build-essential
sudo rosdep init || true
rosdep update
bash install_components.sh
```

`install_components.sh` installs the ROS Humble camera, image, diagnostics, robot localization, PCL, BehaviorTree.CPP, OMPL, Ceres and related packages used by the robot workspace. On Jetson-class devices, it also creates a 4 GB swap file when `/swapfile` is not already present.

### 2. Install Livox-SDK2

Livox ROS Driver 2 links against Livox-SDK2, so install the SDK before building `ws_livox`:

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

Return to the repository root before building the ROS workspaces.

### 3. Build the Livox workspace

The Livox driver provides a helper script for ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
cd ws_livox/src/livox_ros_driver2
./build.sh humble
cd ../../..
source install/setup.bash
```

If you build the whole workspace directly, keep the same ROS environment active:

```bash
source /opt/ros/humble/setup.bash
cd ws_livox
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
source install/setup.bash
cd ..
```

### 4. Build the main robot workspace

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
cd car_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
cd ..
```

For a new terminal, source both workspaces in order:

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
source car_ws/install/setup.bash
```

### 5. RViz configuration

The default Nav2 RViz view is stored at:

```text
car_ws/src/navigation2-humble/nav2_bringup/rviz/nav2_default_view.rviz
```

Use it when launching Nav2 with RViz or open it directly with:

```bash
rviz2 -d car_ws/src/navigation2-humble/nav2_bringup/rviz/nav2_default_view.rviz
```

## Run

### Livox MID360 driver and point cloud bridge

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_map_launch.py
```

### FAST-LIO mapping

```bash
source /opt/ros/humble/setup.bash
source car_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py
```

The default FAST-LIO configuration is:

```text
car_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

### YOLO detection

```bash
source /opt/ros/humble/setup.bash
source car_ws/install/setup.bash
ros2 launch yolo_detector yolo_detect.launch.py enable_vis:=true
```

YOLO configuration and model assets are installed from:

```text
car_ws/src/yolo_detector/config/yolo_params.yaml
car_ws/src/yolo_detector/models/
car_ws/src/yolo_detector/utils/
```

### Mission manager

```bash
source /opt/ros/humble/setup.bash
source car_ws/install/setup.bash
ros2 launch yolo_detector mission_manager.launch.py enable_debug:=false
```

### Integrated autonomous navigation stack

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
source car_ws/install/setup.bash
ros2 launch yolo_detector main_nav.launch.py
```

Common launch arguments:

```bash
ros2 launch yolo_detector main_nav.launch.py \
  use_rviz:=true \
  launch_camera:=true \
  launch_lidar:=true \
  yolo_vis:=true \
  distance_vis:=true \
  sensor_startup_delay:=10.0
```

`main_nav.launch.py` defaults to this map path:

```text
/Map_yaml/circuit/map_huandao.yaml
```

Use `map_yaml_file:=...` to provide the map for the current robot or development machine:

```bash
ros2 launch yolo_detector main_nav.launch.py map_yaml_file:=/absolute/path/to/map.yaml
```

## Runtime Flow

A typical competition run follows this sequence:

1. Start the robot chassis driver through `turn_on_wheeltec_robot`.
2. Publish the static transform between `map` and `camera_init`.
3. Start FAST-LIO for localization or mapping.
4. Start the Nav2 map server and activate it through lifecycle transitions.
5. Start Nav2 navigation after the map server becomes active.
6. Start Livox LiDAR and camera nodes after the configured sensor delay.
7. Run YOLO detection, distance detection and costmap processing.
8. Let mission management and Nav2 coordinate target selection and motion.
9. Apply velocity filtering and obstacle analysis before sending motion commands to the chassis.

## Reproduction Notes

- The integrated system is hardware-dependent. For desktop inspection, build the workspaces first and launch individual modules before running `main_nav.launch.py`.
- `build/`, `install/`, and `log/` are generated by `colcon`.
- Some launch files contain machine-specific absolute paths, such as map and RViz paths. Override them with launch arguments or local configuration before deployment.
- Check serial device names, camera calibration, LiDAR configuration, TF frames, and map paths on the target robot before running the full stack.
- The BEV/IPM pipeline assumes a mostly planar ground surface. Slopes, camera pitch/roll vibration, strong lighting changes, and reflective surfaces can reduce reliability.

## Development Workflow

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
source car_ws/install/setup.bash

# Inspect available packages
ros2 pkg list | grep -E "yolo_detector|fast_lio|livox|wheeltec|costmap|distance"

# Launch individual modules during debugging
ros2 launch yolo_detector yolo_detect.launch.py
ros2 launch fast_lio mapping.launch.py
ros2 launch yolo_detector mission_manager.launch.py
```

For integrated tests, start from `main_nav.launch.py` and disable hardware-dependent parts with launch arguments when testing away from the robot.


<br><br>

# 🇨🇳 中文文档 (Chinese Version)

[🇬🇧 English Version (英文版)](#apex-ros2)

本仓库是一个面向智能车比赛的 ROS 2 综合工程，包含车体底盘驱动、激光雷达与相机接入、建图定位、导航规划、视觉检测、BEV 障碍物感知、代价地图处理和速度控制等模块。

## 👥 团队分工

本项目由团队成员协同开发，核心模块的分工如下：

| 团队成员 | 核心负责方向 | 关联功能包 | 工作亮点 |
| :--- | :--- | :--- | :--- |
| **朱首赫** | 视觉检测、状态决策与 BEV 感知 | `yolo_detector`, `bev_obstacle_detector`, 任务决策流 | 实现数字目标识别与状态机决策，开发基于鸟瞰图 (BEV/IPM) 的底层避障策略，优化了整体感知数据管线。 |
| **冯曦熠** | 导航算法与控制滤波 | `navigation2-humble`, `cmd_vel_tools` | 负责 Nav2 导航栈规划器的调参，设计并实现了速度控制仲裁机制。 |
| **许震宇** | 模型训练与代价地图 | `yolo_detector` (训练侧), `costmap_process` | 负责 YOLO 目标检测模型的训练，开发了代价地图修改机制。 |

## 📜 开源协议与版权声明

### 代码许可
本项目的源代码采用 [MIT License](LICENSE) 开源协议。欢迎各位开发者基于此项目进行学习、修改与二次开发。

### 文档许可与包装声明
本项目的**整体工程化包装、系统架构文档撰写及 README 维护**均由 **朱首赫** 独立完成。

我们非常欢迎大家参考和借鉴本项目的架构设计与代码结构。但在开源分享的同时，也恳请各位开发者尊重原创作者的劳动成果与学术诚信：**请勿在未经授权的情况下，直接将本仓库的整体结构、文档文案或展示排版“原样照搬”并作为个人的独立成果进行展示。** 如果本项目在工程规范或文档架构上对您有所启发，在合理引用的同时注明出处，将是对开源贡献者最大的鼓励与支持。

---

## 技术栈 (Technical Stack)

- **运行环境**: Ubuntu 22.04, ROS 2 Humble, Python 3.10, `colcon`
- **机器人平台**: Ackermann 移动底盘 (搭载 STM32 底层控制器)
- **计算平台**: NVIDIA Jetson Orin Nano 级边缘计算设备
- **传感器**: Livox MID360 激光雷达, Astra Pro Plus 深度相机, USB 摄像头, 底盘里程计/IMU
- **定位建图**: FAST-LIO, AMCL/Nav2 定位, 点云转激光扫描数据转换
- **导航控制**: Nav2 Humble, Smac Hybrid-A* 规划算法, Regulated Pure Pursuit 局部控制, 语义代价地图层
- **视觉感知**: YOLOv8 目标检测, 基于 Mask 的相机距离检测, 基于 GPU 的 BEV/IPM 障碍物处理
- **控制安全**: `cmd_vel` 滤波机制, 停车线状态机, 速度限制与障碍物分析

## 核心功能包 (Main Packages)

| 功能包 (Package) | 作用 (Role) |
| --- | --- |
| `yolo_detector` | YOLOv8 视觉检测、任务管理、状态机决策与系统启动入口 |
| `turn_on_wheeltec_robot` | 底盘驱动、串口通信、TF 坐标树、EKF 滤波及传感器启动文件 |
| `FAST_LIO_ROS2` | 基于 FAST-LIO 的激光惯性里程计建图与定位 |
| `navigation2-humble` | 提供地图服务器、规划控制及生命周期管理的 Nav2 导航栈 |
| `livox_ros_driver2` | Livox 激光雷达 (含 MID360) 的 ROS 2 驱动与配置文件 |
| `bev_obstacle_detector` | 鸟瞰图 (BEV/IPM) 障碍物检测，多点云融合与速度障碍分析 |
| `distance_detector` | 基于相机掩膜 (Mask) 的距离与危险区域检测 |
| `costmap_process` | 语义地图处理、代价地图发布及可视化 |
| `cmd_vel_tools` | 速度滤波、障碍物分析判断与防撞红区处理逻辑 |

> 注：关于完整的编译、运行指南及详细的 Launch 文件说明，由于跨平台配置繁琐，请参考上方 [English Version](#apex-ros2) 中的技术细节描述。

