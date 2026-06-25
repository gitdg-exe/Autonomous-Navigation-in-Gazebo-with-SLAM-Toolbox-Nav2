# 🤖 Dhruv_robot — Autonomous Navigation in Gazebo with SLAM Toolbox & Nav2

> A fully custom-built differential-drive robot simulated in Gazebo Classic 11, equipped with a 360° LiDAR, a 2-DOF robotic arm, and full Nav2 autonomous navigation using SLAM Toolbox for real-time mapping — built on ROS 2 Humble from scratch.

---

## 📋 Table of Contents

- [Demo](#-demo)
- [Overview](#-overview)
- [Features](#-features)
- [Robot Description](#-robot-description-dhruv_robot)
- [System Architecture](#-system-architecture)
- [TF Tree](#-tf-tree)
- [Software Stack](#-software-stack)
- [Package Structure](#-package-structure)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Usage](#-usage)
- [Configuration](#-configuration)
- [World Description](#-world-description)
- [Nav2 Stack Details](#-nav2-stack-details)
- [SLAM Toolbox Details](#-slam-toolbox-details)
- [Known Issues & Troubleshooting](#-known-issues--troubleshooting)
- [License](#-license)

---

## 🎬 Demo

> _Simulation running in Gazebo Classic 11 with RViz2 showing real-time map building, costmaps, and Nav2 goal execution._

| Gazebo Simulation | RViz2 Navigation View |
|:-----------------:|:---------------------:|
| _(screenshot)_ | _(screenshot)_ |

---

## 🧭 Overview

**`project_ow_nav2`** is a ROS 2 Humble package that simulates a custom differential-drive robot — `Dhruv_robot` — in Gazebo Classic 11. The robot autonomously navigates a walled obstacle arena using:

- **SLAM Toolbox** (online async mode) to build a 2D occupancy grid map in real time from LiDAR data
- **Nav2** for global path planning (NavFn/Dijkstra), local trajectory execution (DWB controller), obstacle avoidance, and recovery behaviors
- A hand-crafted **URDF** with full inertial properties, Gazebo friction parameters, and sensor plugins
- A **2-DOF robotic arm** (shoulder + elbow) mounted on the front of the chassis, independently controllable via `/arm_command`

The entire simulation, SLAM, navigation, and visualization stack is launched with a single command using a staggered `TimerAction`-based launch file.

---

## ✨ Features

- **Custom URDF robot** with accurate mass/inertia tensors for all links
- **4-wheel differential drive** via `gazebo_ros_diff_drive` plugin (rear wheels driven)
- **360° LiDAR sensor** — 360 samples, 0.12 m – 8.0 m range, 10 Hz, with Gaussian noise
- **2-DOF robotic arm** — shoulder and elbow revolute joints (±90°)
- **Real-time SLAM** using SLAM Toolbox (online async) with Ceres solver and loop closure
- **Autonomous point-to-point navigation** via Nav2 (set goals in RViz2 with 2D Nav Goal)
- **DWB local planner** with tuned critics (PathAlign, GoalDist, RotateToGoal, etc.)
- **Layered costmaps** — global (static + obstacle + inflation) and local (voxel + inflation)
- **Custom obstacle world** — 10 m × 10 m walled arena with randomized boxes and cylinders
- **Staggered launch sequencing** to prevent race conditions between Gazebo, SLAM, and Nav2
- **No AMCL** — SLAM Toolbox directly provides the `map → odom` transform

---

## 🦾 Robot Description: `Dhruv_robot`

The robot is fully defined in `urdf/Dhruv.urdf`.

### Physical Dimensions

| Component | Value |
|-----------|-------|
| Chassis (L × W × H) | 0.40 m × 0.28 m × 0.10 m |
| Wheel radius | 0.075 m |
| Wheel width | 0.04 m |
| Wheel separation (track) | 0.35 m |
| Wheelbase (front-to-rear axle) | 0.28 m |
| Chassis ground clearance | 0.075 m |
| LiDAR height from ground | ~0.375 m |

### Mass Distribution

| Link | Mass |
|------|------|
| `base_link` (chassis) | 5.0 kg |
| Each wheel (× 4) | 0.3 kg |
| `arm_base_link` | 0.15 kg |
| `upper_arm_link` | 0.40 kg |
| `forearm_link` | 0.25 kg |
| `lidar_link` | 0.10 kg |
| **Total (approx.)** | **~7.1 kg** |

### Link Tree

```
base_footprint
└── base_link  [fixed, z+0.075]
    ├── front_left_wheel_joint   [continuous, y-axis]
    ├── front_right_wheel_joint  [continuous, y-axis]
    ├── rear_left_wheel_joint    [continuous, y-axis]  ← driven
    ├── rear_right_wheel_joint   [continuous, y-axis]  ← driven
    ├── arm_base_joint           [fixed, front top]
    │   └── shoulder_joint       [revolute ±90°, y-axis]
    │       └── elbow_joint      [revolute ±90°, y-axis]
    └── lidar_joint              [fixed, z+0.30 from base_link]
```

### Gazebo Plugins

| Plugin | Function |
|--------|----------|
| `libgazebo_ros_diff_drive.so` | Subscribes `/cmd_vel`, publishes `/odom`, drives rear wheels |
| `libgazebo_ros_joint_state_publisher.so` | Publishes all 6 joint states to `/joint_states` |
| `libgazebo_ros_ray_sensor.so` | Publishes 360° LiDAR scan to `/scan` |

### Wheel Friction Notes

The front wheels have `mu1=mu2=0.0` (free-rolling, low resistance) while the rear driven wheels have `mu1=mu2=0.4` (traction). This asymmetry reflects a real 4-wheel skid-steer where only the rear axle is powered.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo Classic 11                         │
│   ┌──────────────┐   /scan      ┌──────────────────────────┐   │
│   │  LiDAR       │ ──────────►  │                          │   │
│   │  Plugin      │              │   SLAM Toolbox            │   │
│   └──────────────┘              │   (online_async)          │   │
│                                 │   map → odom TF           │   │
│   ┌──────────────┐   /odom      └──────────┬───────────────┘   │
│   │  Diff Drive  │ ──────────►             │  /map              │
│   │  Plugin      │                         ▼                    │
│   │              │ ◄──────── /cmd_vel  ┌───────────────────┐   │
│   └──────────────┘                     │   Nav2 Stack       │   │
│                                        │                    │   │
│   ┌──────────────┐  /joint_states      │  ┌─────────────┐  │   │
│   │  Joint State │ ──────────►         │  │ NavFn       │  │   │
│   │  Publisher   │                     │  │ Global Plan │  │   │
│   └──────────────┘                     │  └──────┬──────┘  │   │
│                                        │         │         │   │
│   ┌──────────────┐  robot_description  │  ┌──────▼──────┐  │   │
│   │  Robot State │ ──────────►         │  │ DWB Local   │  │   │
│   │  Publisher   │  odom→base_fp TF    │  │ Controller  │  │   │
│   └──────────────┘                     │  └─────────────┘  │   │
│                                        └───────────────────┘   │
│                                                                  │
│                        RViz2 (visualization)                     │
│           2D Nav Goal ──────────────────────────────────►       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🌳 TF Tree

```
map
 └── odom                  (published by SLAM Toolbox)
      └── base_footprint   (published by diff_drive plugin)
           └── base_link
                ├── front_left_wheel
                ├── front_right_wheel
                ├── rear_left_wheel
                ├── rear_right_wheel
                ├── lidar_link
                └── arm_base_link
                     └── upper_arm_link
                          └── forearm_link
```

SLAM Toolbox owns the `map → odom` edge. The diff drive plugin owns `odom → base_footprint`. Robot State Publisher owns all static/kinematic transforms below `base_footprint`.

---

## 🛠️ Software Stack

| Component | Package / Version |
|-----------|-------------------|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble Hawksbill |
| Simulator | Gazebo Classic 11 |
| SLAM | SLAM Toolbox (online async, Ceres solver) |
| Global Planner | NavFn (Dijkstra) |
| Local Controller | DWB (Dynamic Window-Based) |
| Path Smoother | Nav2 SimpleSmoother |
| Costmaps | Nav2 Costmap2D (global + local) |
| Visualization | RViz2 |
| Build System | ament_cmake / colcon |

---

## 📁 Package Structure

```
project_ow_nav2/
├── urdf/
│   └── Dhruv.urdf              # Full robot description (links, joints, Gazebo plugins)
├── config/
│   ├── nav2_params.yaml        # Full Nav2 parameter file (AMCL, BT, controller, costmaps)
│   └── slam_toolbox_params.yaml  # SLAM Toolbox online_async configuration
├── worlds/
│   └── obstacles.world         # 10×10 m SDF arena with walls + randomized obstacles
├── launch/
│   └── gazebo_launch.py        # Single-entry-point launch with staggered TimerActions
├── maps/                       # (empty) — for saving maps with map_saver_cli
├── include/
│   └── project_ow_nav2/        # C++ headers (reserved for future plugins)
├── CMakeLists.txt
├── package.xml
└── LICENSE                     # Apache-2.0
```

---

## ✅ Prerequisites

Make sure the following are installed on your system:

```bash
# ROS 2 Humble (full desktop)
sudo apt install ros-humble-desktop

# Gazebo Classic 11 + ROS bridge
sudo apt install ros-humble-gazebo-ros-pkgs

# Navigation stack
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# RViz2 (included in desktop, but just in case)
sudo apt install ros-humble-rviz2
```

> **Note:** `gazebo_ros2_control` is **not used** in this project. Drive and joint state publishing are handled entirely by the `gazebo_ros_diff_drive` and `gazebo_ros_joint_state_publisher` Gazebo plugins embedded in the URDF.

---

## 📦 Installation

```bash
# 1. Create (or navigate to) your ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone this repository
git clone https://github.com/<your-username>/project_ow_nav2.git

# 3. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install

# 5. Source the workspace
source install/setup.bash
```

Add the source line to your `~/.bashrc` to avoid running it every session:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Usage

### Launch Everything (Single Command)

```bash
ros2 launch project_ow_nav2 gazebo_launch.py
```

This launches the following in order (with built-in delays to avoid race conditions):

| Delay | What Starts |
|-------|------------|
| 0 s | Robot State Publisher, Gazebo server |
| 3 s | Gazebo client (GUI), robot spawned as `Dhruv_robot` |
| 8 s | SLAM Toolbox (online async), Nav2 stack |
| 12 s | RViz2 with Nav2 default view |

### Sending a Navigation Goal

Once all nodes are running and the map has started building:

1. In RViz2, click **"2D Nav Goal"** in the toolbar
2. Click and drag on the map to set the target pose
3. Nav2 will compute a global path using NavFn and execute it using the DWB controller

### Manually Controlling the Robot

```bash
# Install teleop if needed
sudo apt install ros-humble-teleop-twist-keyboard

# Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Controlling the Arm

The arm joints are driven by publishing to `/arm_command`:

```bash
# Raise the shoulder joint to 45 degrees
ros2 topic pub /arm_command std_msgs/msg/Float64MultiArray \
  "data: [0.785, 0.0]"
```

The array index corresponds to `[shoulder_joint, elbow_joint]` in radians.

### Saving the Built Map

Once you've navigated and built a complete map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/project_ow_nav2/maps/my_map
```

This saves `my_map.pgm` and `my_map.yaml` to the `maps/` directory.

### Monitoring Key Topics

```bash
# Check LiDAR
ros2 topic echo /scan --no-arr

# Check odometry
ros2 topic echo /odom

# Check velocity commands going to robot
ros2 topic hz /cmd_vel

# View TF tree
ros2 run tf2_tools view_frames

# Check Nav2 action server
ros2 action list
```

---

## ⚙️ Configuration

### `config/slam_toolbox_params.yaml`

Key parameters:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `mode` | `mapping` | Online mapping (not localization) |
| `solver_plugin` | `CeresSolver` | Pose graph optimizer |
| `scan_topic` | `/scan` | Input LiDAR topic |
| `map_update_interval` | `5.0` | Map publish interval (seconds) |
| `resolution` | `0.05` | Map cell size (5 cm/pixel) |
| `max_laser_range` | `8.0` | Match LiDAR URDF max range |
| `tf_buffer_duration` | `60.0` | Prevent stale TF errors on startup |
| `transform_publish_period` | `0.02` | 50 Hz TF publishing |
| `do_loop_closing` | `true` | Enable loop closure correction |
| `minimum_travel_distance` | `0.5` | Min distance before new scan is accepted |

### `config/nav2_params.yaml` — Key Sections

**Controller Server (DWB):**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `max_vel_x` | `0.5 m/s` | Max forward speed |
| `max_vel_theta` | `1.5 rad/s` | Max rotation speed |
| `sim_time` | `2.5 s` | Trajectory rollout horizon |
| `vx_samples` | `20` | Linear velocity samples |
| `vtheta_samples` | `40` | Angular velocity samples |
| `xy_goal_tolerance` | `0.25 m` | Goal position tolerance |
| `yaw_goal_tolerance` | `0.25 rad` | Goal heading tolerance |

**DWB Critics (local planner scoring):**

| Critic | Scale | Role |
|--------|-------|------|
| `RotateToGoal` | 32.0 | Aligns heading before final approach |
| `PathDist` | 32.0 | Penalizes deviation from global path |
| `PathAlign` | 32.0 | Keeps robot aligned with path direction |
| `GoalDist` | 24.0 | Drives toward goal position |
| `GoalAlign` | 24.0 | Aligns with goal orientation |
| `BaseObstacle` | 0.02 | Penalizes proximity to obstacles |
| `Oscillation` | — | Prevents back-and-forth oscillation |

**Costmaps:**

| Costmap | Layers | Frame |
|---------|--------|-------|
| Global | `static_layer`, `obstacle_layer`, `inflation_layer` | `map` |
| Local | `voxel_layer`, `inflation_layer` | `odom` (rolling 3×3 m window) |

Both costmaps use `inflation_radius: 0.4 m` with `cost_scaling_factor: 3.0`.

---

## 🌍 World Description

**File:** `worlds/obstacles.world`

The simulation environment is a **10 m × 10 m** walled arena (`wall_north`, `wall_south`, `wall_east`, `wall_west`) containing a mix of randomized static obstacles:

- Multiple **box obstacles** of varying dimensions (0.3–0.6 m tall), placed at arbitrary rotations
- Multiple **cylinder obstacles** (0.3 m radius, 0.6 m tall)

All obstacles are static (`<static>true</static>`) and have distinct colors for visual clarity in Gazebo. Physics uses ODE with `real_time_update_rate: 1000` and `max_step_size: 0.001`.

---

## 🧠 Nav2 Stack Details

The Nav2 stack is launched using `nav2_bringup/navigation_launch.py` **without** AMCL — SLAM Toolbox serves as the sole localization source by directly broadcasting `map → odom`.

### Global Planning
NavFn with Dijkstra's algorithm (`use_astar: false`) is used for computing the global path from the robot's current pose to the goal. The planner tolerates unknown space (`allow_unknown: true`), enabling navigation while the map is still being built.

### Local Control
The DWB (Dynamic Window-Based) controller samples trajectories across the `(vx, vtheta)` space and scores them using seven critics. Trajectories are simulated 2.5 seconds forward and sampled at 20 linear × 40 angular velocity pairs. The best-scoring trajectory is executed.

### Recovery Behaviors
If the robot gets stuck, Nav2 will attempt the following recovery sequence in order:
1. **Spin** — rotate in place to clear local costmap
2. **Backup** — reverse a short distance
3. **Drive on Heading** — push forward along current heading
4. **Wait** — pause and wait for dynamic obstacles to clear

### Behavior Tree
Navigation uses the default `NavigateToPose` behavior tree from Nav2, managed by `nav2_bt_navigator`.

---

## 📡 SLAM Toolbox Details

SLAM Toolbox runs in **online async** mode — it processes LiDAR scans asynchronously as they arrive, which means it doesn't block the main control loop. Key design choices:

- **Ceres Solver** (`SPARSE_NORMAL_CHOLESKY`) is used for pose graph optimization — more accurate than the default Karto solver
- **Loop closure** is enabled with a 3.0 m search radius, so the map self-corrects when the robot revisits areas
- `transform_publish_period: 0.02` (50 Hz) ensures Nav2's TF lookups never time out
- `tf_buffer_duration: 60.0` on the SLAM node prevents "extrapolation into the past" errors during the startup delay

---

## 🔧 Known Issues & Troubleshooting

### Robot only rotates, doesn't move forward
This is typically a DWB critic misconfiguration. Check:
```bash
ros2 topic echo /cmd_vel
```
If `linear.x` is always 0.0, the `RotateToGoal` critic may be dominating. Lower `RotateToGoal.scale` (e.g., to `16.0`) and increase `PathDist.scale`.

### `map → odom` TF not available
SLAM Toolbox starts 8 seconds after launch, so Nav2 may log TF errors briefly. These should resolve within ~2 seconds of SLAM starting. If they persist:
```bash
ros2 run tf2_tools view_frames
```
Look for a broken chain at `map → odom`. Check that `/scan` is publishing at ~10 Hz.

### Gazebo spawns but robot falls through the floor
This can happen if the physics engine starts before the URDF is fully loaded. The 3-second `TimerAction` delay before spawning should prevent this. If it persists, increase the delay to 5 seconds.

### `stale TF` warnings from Nav2 costmap
Increase `transform_tolerance` in `nav2_params.yaml` for the costmap layer causing the warning, or increase `transform_publish_period` in `slam_toolbox_params.yaml` to publish TF at a higher rate.

### LiDAR scan not appearing in RViz2
Make sure RViz2 fixed frame is set to `map` (or `odom` if SLAM hasn't started yet). Add a LaserScan display and set the topic to `/scan`.

---

## 📄 License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgements

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo ROS Packages](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
