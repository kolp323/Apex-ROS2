# Apex-ROS2

Apex-ROS2 is a ROS 2 Humble intelligent vehicle workspace for autonomous driving competition scenarios. The project integrates chassis control, Livox MID360 LiDAR, Astra/USB camera drivers, FAST-LIO mapping, Nav2 navigation, YOLO-based perception, BEV obstacle detection, semantic costmap processing, velocity limiting, and custom ROS 2 messages into a complete robot software stack.

本仓库是一个面向智能车比赛的 ROS 2 综合工程，包含车体底盘驱动、激光雷达与相机接入、建图定位、导航规划、视觉检测、障碍物感知、代价地图处理和速度控制等模块。

## Project Highlights

- **ROS 2 Humble multi-workspace architecture**: separates the main vehicle stack and Livox-related packages into `car_ws` and `ws_livox`.
- **Competition-oriented autonomous navigation**: combines FAST-LIO localization/mapping, Nav2 map server/navigation, task management, and robot base control.
- **Multi-sensor perception**: integrates Livox MID360 LiDAR, Astra camera, USB camera, YOLO object detection, distance detection, BEV obstacle detection, and point cloud processing.
- **Custom decision and safety modules**: includes mission management, semantic costmap publishing, obstacle analysis, `cmd_vel` limiting, and custom message packages.
- **Hardware-oriented deployment layout**: launch files and configs are organized around the actual Wheeltec-style mobile robot platform and competition runtime.

## Repository Structure

```text
Apex-ROS2/
├── car_ws/                  # Main ROS 2 workspace for robot, perception, navigation and control
│   └── src/
│       ├── bev_obstacle_detector/     # BEV/IPM obstacle detection and point cloud fusion
│       ├── cmd_vel_tools/             # Velocity limiting, obstacle analysis and red segment processing
│       ├── costmap_process/           # Semantic map and costmap processing nodes
│       ├── distance_detector/         # Mask-based visual proximity/distance detection
│       ├── distance_detector_msg/     # Custom distance detection messages
│       ├── FAST_LIO_ROS2/             # FAST-LIO LiDAR-inertial mapping integration
│       ├── navigation2-humble/        # Nav2 Humble navigation stack
│       ├── red_segment_msg/           # Custom red segment messages
│       ├── robot_kcf/                 # Visual tracking related package
│       ├── ros2_astra_camera-master/  # Astra camera ROS 2 driver
│       ├── script/                    # Auxiliary scripts
│       ├── serial_ros2/               # Serial communication dependency package
│       ├── turn_on_wheeltec_robot/    # Robot chassis driver, TF, EKF and sensor launch files
│       ├── usb_cam-ros2/              # USB camera ROS 2 driver
│       ├── wheeltec_robot_msg/        # Custom robot messages
│       ├── wheeltec_robot_urdf/       # Robot URDF, meshes and RViz description files
│       └── yolo_detector/             # YOLO detection, mission manager and system launch entrypoints
├── ws_livox/                 # Livox LiDAR workspace
│   └── src/
│       ├── livox_ros_driver2/          # Livox ROS 2 driver and MID360 launch/config files
│       └── pointcloud_to_laserscan-humble/ # PointCloud2 to LaserScan conversion
└── README.md
```

## System Architecture

Apex-ROS2 is organized as a layered autonomous vehicle stack:

1. **Hardware and sensor layer**
   - `turn_on_wheeltec_robot` connects to the chassis controller through serial communication and publishes odometry-related data.
   - `livox_ros_driver2` publishes Livox MID360 LiDAR data.
   - `ros2_astra_camera-master` and `usb_cam-ros2` provide camera inputs.
   - `wheeltec_robot_urdf` provides robot model, meshes and visualization configuration.

2. **Localization and mapping layer**
   - `FAST_LIO_ROS2` runs FAST-LIO mapping with `mid360.yaml` as the default configuration.
   - A static transform connects `map` and `camera_init` in the main navigation launch flow.
   - `pointcloud_to_laserscan-humble` bridges point cloud data into laser scan format when required by downstream modules.

3. **Perception layer**
   - `yolo_detector` runs YOLO-based visual detection and exposes `yolo_node`.
   - `distance_detector` detects objects in a masked proximity area and provides `distance_node` / `distance_node_cv`.
   - `bev_obstacle_detector` provides inverse-perspective / BEV obstacle processing through `ipm_node`, `pointcloud_merger_node`, and `ipm_cmd_vel_node`.
   - `cmd_vel_tools` includes red segment processing and obstacle analysis utilities.

4. **Navigation and decision layer**
   - `navigation2-humble` provides the Nav2 planner, controller, behavior tree navigation, map server and lifecycle infrastructure.
   - `yolo_detector` includes `mission_manager`, which coordinates detection results and navigation targets.
   - `costmap_process` publishes semantic/costmap information through nodes such as `senamic_node`, `costmap_pub_node`, `map_vis_publisher`, and `costmap_vis_publisher`.

5. **Control and safety layer**
   - Nav2 outputs velocity commands.
   - `cmd_vel_tools` and BEV/distance modules can constrain or analyze velocity commands before they reach the robot base.
   - `turn_on_wheeltec_robot` sends final control commands to the chassis.

## Main Packages

| Package | Workspace | Role |
| --- | --- | --- |
| `yolo_detector` | `car_ws` | YOLO perception, mission management, decision-related launch files and full-system startup entrypoints |
| `turn_on_wheeltec_robot` | `car_ws` | Wheeltec robot chassis driver, serial communication, TF, EKF and sensor startup launch files |
| `FAST_LIO_ROS2` / `fast_lio` | `car_ws` | LiDAR-inertial mapping and localization based on FAST-LIO |
| `navigation2-humble` | `car_ws` | Nav2 navigation stack for map server, planning, control and lifecycle management |
| `livox_ros_driver2` | `ws_livox` | ROS 2 driver and launch/config files for Livox LiDARs, including MID360 |
| `pointcloud_to_laserscan-humble` | `ws_livox` | Converts point clouds to laser scan messages |
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

## Environment

The code is organized for a ROS 2 Humble environment, typically on Ubuntu 22.04 or an equivalent robot runtime.

Recommended base environment:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `colcon`
- Nav2 dependencies
- Livox SDK / Livox ROS 2 driver requirements
- Camera driver dependencies for Astra / USB camera modules
- Python dependencies used by perception modules, such as OpenCV, NumPy, PyTorch/YOLO-related packages, depending on the deployment machine

Hardware used by the integrated launch files includes:

- Wheeltec-style mobile robot chassis
- Livox MID360 LiDAR
- Astra camera and/or USB camera
- Robot serial controller, defaulting to `/dev/wheeltec_controller` in `base_serial.launch.py`

## Build

Build the Livox workspace first, then build the main vehicle workspace.

```bash
cd ws_livox
colcon build --symlink-install
source install/setup.bash
```

```bash
cd ../car_ws
colcon build --symlink-install
source install/setup.bash
```

When launching the integrated system from a new terminal, source both workspaces in order:

```bash
source ws_livox/install/setup.bash
source car_ws/install/setup.bash
```

If your ROS 2 installation is not already sourced, source it first:

```bash
source /opt/ros/humble/setup.bash
```

## Run

### 1. Livox MID360 driver and point cloud bridge

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_map_launch.py
```

### 2. FAST-LIO mapping

```bash
source /opt/ros/humble/setup.bash
source car_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py
```

The default FAST-LIO launch file uses:

```text
car_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

### 3. YOLO detection

```bash
source /opt/ros/humble/setup.bash
source car_ws/install/setup.bash
ros2 launch yolo_detector yolo_detect.launch.py enable_vis:=true
```

The YOLO package installs models and utilities from:

```text
car_ws/src/yolo_detector/models/
car_ws/src/yolo_detector/utils/
car_ws/src/yolo_detector/config/yolo_params.yaml
```

### 4. Mission manager

```bash
source /opt/ros/humble/setup.bash
source car_ws/install/setup.bash
ros2 launch yolo_detector mission_manager.launch.py enable_debug:=false
```

### 5. Integrated autonomous navigation stack

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

`main_nav.launch.py` defaults to the following map path:

```text
/Map_yaml/circuit/map_huandao.yaml
```

Use `map_yaml_file:=...` to provide the map for your deployment machine:

```bash
ros2 launch yolo_detector main_nav.launch.py map_yaml_file:=/absolute/path/to/map.yaml
```

## Runtime Flow

A typical competition run follows this sequence:

1. Start the robot chassis driver through `turn_on_wheeltec_robot`.
2. Publish the static transform between `map` and `camera_init`.
3. Start FAST-LIO for localization/mapping.
4. Start the Nav2 map server and activate it through lifecycle transitions.
5. Start Nav2 navigation after the map server becomes active.
6. Start Livox LiDAR and camera nodes after the configured sensor delay.
7. Run YOLO detection, distance detection and costmap processing.
8. Let mission management and Nav2 coordinate target selection and motion.
9. Apply velocity limiting and obstacle analysis before sending motion commands to the chassis.

## Notes

- `build/`, `install/`, and `log/` are generated by `colcon` and are intentionally excluded from this repository.
- The repository preserves integrated upstream components such as Nav2, FAST-LIO, Livox driver and camera drivers because this competition workspace depends on their local package layout.
- Some launch files contain machine-specific absolute paths, such as map and RViz paths. Update those paths through launch arguments or local configuration before deployment.
- Hardware device names, camera parameters and LiDAR configuration should be checked on the target robot before running the full stack.

## Suggested Development Workflow

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

For integrated tests, start from `main_nav.launch.py` and disable hardware-dependent parts with launch arguments when testing on a non-robot machine.
