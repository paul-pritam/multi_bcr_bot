# Multi-Robot Navigation Stack (bcr_bot)

![Multi-Robot Warehouse Simulation and RViz Navigation](res/multi_robot_demo.png)

Fork of [blackcoffeerobotics/bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot) extended with a **multi-robot simulation and navigation system** on ROS 2 Jazzy + Gazebo Harmonic.

Spawns N differential-drive robots in a shared warehouse environment, each with independent namespaced Nav2 stacks, EKF localization, and AMCL; all launched from a single YAML config.

## What I built

| Component | File | What it does |
|---|---|---|
| Multi-robot spawner | `launch/multi_bcr_bot.launch.py` | Reads `config/robots.yaml`, spawns each robot in Gazebo with its own `ros_gz_bridge`, `robot_state_publisher`, and static TFs under a ROS 2 namespace |
| Per-robot Nav2 | `launch/multi_nav2.launch.py` | Generates per-robot Nav2 params from `config/nav2_template.yaml` using `<ROBOT_NAME>` substitution, staggers launch with `TimerAction` |
| Composable Nav2 | `launch/multi_nav2_composed.launch.py` | Same as above but loads Nav2 nodes as composable components into an isolated container for lower memory usage |
| Multi-robot localization | `launch/localization.launch.py` | Spawns per-robot EKF (`robot_localization`) + AMCL with correctly namespaced frames (`{ns}/odom`, `{ns}/base_footprint`), shared map server |
| Nav2 template | `config/nav2_template.yaml` | Full Nav2 configuration (DWB planner, costmaps, collision monitor, velocity smoother, docking) parameterized by `<ROBOT_NAME>` |
| EKF config | `config/ekf.yaml` | 2D EKF fusing wheel odometry + IMU |
| AMCL config | `config/amcl.yaml` | AMCL with likelihood field model, tuned particle counts |
| Robot config | `config/robots.yaml` | Define robots: name, spawn position (x, y, yaw). Add more robots by adding entries |

### Architecture

```
config/robots.yaml
        |
        v
multi_bcr_bot.launch.py
        |
        +-- Gazebo (shared, single instance)
        |
        +-- For each robot in robots.yaml:
        |     +-- robot_state_publisher (namespaced)
        |     +-- ros_gz_bridge (per-robot sensor topics)
        |     +-- Gazebo spawn at (x, y, yaw)
        |     +-- Static TF publishers
        |
        +-- localization.launch.py (delayed 7s)
        |     +-- Per-robot: EKF + AMCL + lifecycle_manager
        |     +-- Shared: map_server + lifecycle_manager
        |
        +-- multi_nav2.launch.py
              +-- Per-robot: controller_server, planner_server,
                  behavior_server, bt_navigator, lifecycle_manager
                  (each in its own namespace)
```

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces
```

## Build

```bash
cd ~/bcr_ws
colcon build --packages-select bcr_bot
source install/setup.bash
```

## Run

### Single robot (original behavior)

```bash
ros2 launch bcr_bot gz.launch.py
```

### Multi-robot

```bash
ros2 launch bcr_bot multi_bcr_bot.launch.py
```

This launches 2 robots (default config) in a warehouse. Edit `config/robots.yaml` to add more:

```yaml
robots:
  - name: bcr_bot_1
    x: 0.0
    y: 0.0
    yaw: 0.0
  - name: bcr_bot_2
    x: 0.0
    y: 2.0
    yaw: 0.0
  - name: bcr_bot_3
    x: 4.0
    y: 0.0
    yaw: 1.57
```

### Multi-robot with Nav2

```bash
ros2 launch bcr_bot multi_bcr_bot.launch.py
# In a separate terminal, after Gazebo is up:
ros2 launch bcr_bot multi_nav2.launch.py
```

Or use the composable version for lower overhead:

```bash
ros2 launch bcr_bot multi_nav2_composed.launch.py
```

### Mapping (SLAM Toolbox)

```bash
ros2 launch bcr_bot mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/bcr_bot_1/cmd_vel
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f src/bcr_bot/config/bcr_map
```

## Launch arguments

```bash
ros2 launch bcr_bot gz.launch.py \
    camera_enabled:=true \
    stereo_camera_enabled:=false \
    two_d_lidar_enabled:=true \
    world_file:=small_warehouse.sdf
```

## Namespace structure

Every topic, TF, and service is under `/{robot_name}/`:

```
/bcr_bot_1/odom
/bcr_bot_1/scan
/bcr_bot_1/cmd_vel
/bcr_bot_1/tf
/bcr_bot_2/odom
/bcr_bot_2/scan
...
```

The shared `/map` topic is published once by the map server and consumed by all robots' AMCL instances.

## Based on

- [blackcoffeerobotics/bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot) - original single-robot simulation
- [Nav2](https://docs.nav2.org/) - navigation framework
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization) - EKF sensor fusion
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) - online mapping

## License

Apache License 2.0
