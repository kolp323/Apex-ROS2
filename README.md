# Apex-ROS2

Apex-ROS2 is a ROS 2 Humble workspace for an autonomous Ackermann robot used in intelligent vehicle competition scenarios. It combines chassis control, Livox MID360 LiDAR, Astra/USB cameras, FAST-LIO mapping, Nav2 navigation, YOLOv8 perception, BEV obstacle processing, semantic costmap updates, mission management, and velocity arbitration into one deployable robot stack.

本仓库是一个面向智能车比赛的 ROS 2 综合工程，包含车体底盘驱动、激光雷达与相机接入、建图定位、导航规划、视觉检测、BEV 障碍物感知、代价地图处理和速度控制等模块。

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

## Environment

Recommended base environment:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `colcon`
- Nav2 dependencies
- Livox SDK / Livox ROS 2 driver dependencies
- Astra and USB camera driver dependencies
- Python perception dependencies such as OpenCV, NumPy, PyTorch and YOLO-related packages

Hardware expected by the integrated launch files:

- Ackermann mobile robot chassis
- STM32-based chassis controller
- Livox MID360 LiDAR
- Astra camera and/or USB camera
- Robot serial controller, defaulting to `/dev/wheeltec_controller` in `base_serial.launch.py`

## Build

Source ROS 2 first if it is not already active:

```bash
source /opt/ros/humble/setup.bash
```

Build the Livox workspace first:

```bash
cd ws_livox
colcon build --symlink-install
source install/setup.bash
```

Then build the main robot workspace:

```bash
cd ../car_ws
colcon build --symlink-install
source install/setup.bash
```

For a new terminal, source both workspaces in order:

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
source car_ws/install/setup.bash
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
- `build/`, `install/`, and `log/` are generated by `colcon` and are intentionally excluded from the repository.
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
