# ROS 2 Companion Robot

A mobile companion-rover simulation built with ROS 2 Jazzy and Gazebo
Harmonic. The project covers the robot model, differential-drive motion, 2D
LiDAR, a rear RGB camera, manual control, odometry, SLAM, and a saved occupancy
map.

The current milestone is autonomous localization, navigation, waypoint patrol,
energy-aware docking, and mission-level recovery with Nav2.

## Features

- Parametric robot model written in URDF/Xacro
- Differential-drive physics in Gazebo Sim
- Two drive wheels and front/rear caster support
- Simulated 360-degree 2D LiDAR
- Simulated rear RGB camera with ROS image and calibration topics
- Safe terminal teleoperation using `W`, `A`, `S`, and `D`
- Gazebo pose odometry on `/odom`
- Encoder-style wheel odometry on `/wheel_odom` for comparison
- Online mapping with SLAM Toolbox
- RViz configurations for the robot, simulation, and mapping
- Reusable 8 x 6 metre arena with three static obstacles
- Saved map and initial Nav2 localization/navigation configuration
- Forward and limited reverse motion during autonomous navigation
- Configurable multi-waypoint patrol behaviour
- Return-to-home service that safely interrupts an active patrol
- Reverse-entry docking station with LiDAR-protected automatic docking
- ArUco dock-marker detection and camera-guided final alignment
- Safe stop and bounded recovery when the dock marker is obscured or lost
- Energy-aware patrol with automatic low-battery docking, charging, undocking,
  and waypoint resumption
- Moving pedestrian obstacle detected through LiDAR and the Nav2 costmaps
- Nav2 360-degree collision slowdown and emergency-stop zones
- Central mission state manager with automatic patrol and docking recovery

## Current Status

| Capability | Status |
| --- | --- |
| Robot model and RViz visualization | Complete |
| Gazebo arena and robot spawning | Complete |
| Differential-drive control | Complete |
| LiDAR and odometry bridge | Complete |
| Rear RGB camera and ROS image bridge | Complete |
| Dock-marker perception | Complete |
| SLAM mapping and map export | Complete |
| Nav2 localization and autonomous navigation | Initial implementation |
| Multi-waypoint patrol | Initial implementation |
| Return to home | Initial implementation |
| Docking station and docking poses | Complete |
| Camera-guided precision auto-docking | Complete in simulation |
| Marker-loss safety and recovery | Initial implementation |
| Automatic undocking | Initial implementation |
| Battery and charging simulation | Initial implementation |
| Low-battery docking trigger | Initial implementation |
| Energy-aware patrol pause and resume | Initial implementation |
| Dynamic obstacle avoidance | Initial implementation |
| Mission state and autonomous recovery | Initial implementation |
| Physical robot deployment | Planned |

## Project Structure

```text
.
|-- src/
|   |-- companion_robot_behaviors/
|   |   |-- config/        # Patrol, docking, battery, and mission parameters
|   |   |-- launch/        # Autonomous behavior launch files
|   |   `-- scripts/       # Patrol, docking, battery, and mission nodes
|   |-- companion_robot_description/
|   |   |-- launch/        # Standalone robot visualization
|   |   |-- rviz/          # RViz model configuration
|   |   `-- urdf/          # Parametric robot model
|   |-- companion_robot_gazebo/
|   |   |-- config/        # SLAM Toolbox parameters
|   |   |-- launch/        # Simulation and mapping launch files
|   |   |-- maps/          # Saved occupancy maps
|   |   |-- rviz/          # Simulation and mapping views
|   |   |-- scripts/       # WASD teleoperation node
|   |   `-- worlds/        # Gazebo arena
|   |-- companion_robot_perception/
|   |   |-- config/        # Dock-marker detector parameters
|   |   `-- scripts/       # Rear-camera marker detection node
|   `-- companion_robot_navigation/
|       |-- config/        # AMCL, costmap, planner, and controller parameters
|       `-- launch/        # Autonomous-navigation launch file
|-- .gitignore
|-- LICENSE
`-- README.md
```

## Requirements

- Ubuntu 24.04, either native or through WSL 2 with WSLg
- ROS 2 Jazzy
- Gazebo Harmonic and the ROS-Gazebo integration packages
- `colcon` and `rosdep`

All ROS package dependencies are declared in each package's `package.xml` file.

## Installation

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash

# Run these two rosdep commands once per machine.
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

If `rosdep` has already been initialized, skip `sudo rosdep init`.

Each new terminal must source both ROS 2 and this workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Usage

### Display the Robot in RViz

```bash
ros2 launch companion_robot_description display.launch.py
```

On a WSLg system that renders the robot with incorrect dark materials, enable
software rendering:

```bash
ros2 launch companion_robot_description display.launch.py \
  rviz_software_rendering:=true
```

### Start the Gazebo Simulation

```bash
ros2 launch companion_robot_gazebo arena.launch.py
```

This starts Gazebo, spawns the robot, bridges the simulation topics to ROS 2,
and opens the simulation RViz configuration.

### Drive with WASD

Keep the simulation running and open another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run companion_robot_gazebo wasd_teleop
```

| Key | Action |
| --- | --- |
| `W` | Move forward |
| `S` | Move backward |
| `A` | Rotate left |
| `D` | Rotate right |
| `Space` | Stop |
| `Q` | Stop and exit |

The teleoperation node includes a command timeout. The robot automatically
stops when keyboard input is interrupted.

### Build a Map with SLAM

The mapping launch file starts the simulation, SLAM Toolbox, and RViz:

```bash
ros2 launch companion_robot_gazebo mapping.launch.py
```

Run the WASD controller in a second terminal and drive around the arena. For a
clean map, move slowly, avoid collisions, scan the perimeter, and revisit the
starting area so SLAM can perform loop closure.

### Save a Map

Stop the robot, wait briefly for the final scan to be processed, then save the
current `/map` occupancy grid:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "$PWD/src/companion_robot_gazebo/maps/companion_arena"
```

This produces:

```text
src/companion_robot_gazebo/maps/companion_arena.pgm
src/companion_robot_gazebo/maps/companion_arena.yaml
```

The map can also be saved from the **Save Map** row in the SLAM Toolbox RViz
panel. Use **Serialize Map** instead when the pose graph must be saved for
continuing or refining a future mapping session.

After adding a new map, rebuild the Gazebo package so it is included in the
installed package share:

```bash
colcon build --symlink-install --packages-select companion_robot_gazebo
source install/setup.bash
```

### Start Autonomous Navigation

The navigation launch file starts Gazebo, loads the saved map, localizes the
robot with AMCL, starts the Nav2 servers, and opens RViz:

```bash
ros2 launch companion_robot_navigation navigation.launch.py
```

The simulated robot always spawns at `(0, 0, 0)`, so AMCL is initialized with
that pose automatically. If the pose needs correction, select **2D Pose
Estimate** in RViz and drag an arrow from the robot's actual location in its
forward direction. Then select **Nav2 Goal** and place a goal inside the free
area of the map.

Do not run the WASD controller while Nav2 is controlling the robot because both
nodes publish velocity commands to `/cmd_vel`.

The Nav2 local controller may drive backward at a limited speed when a safe
reverse path is more practical. It can still turn and drive forward whenever
that produces the safer or lower-cost route.

### Start Waypoint Patrol

The patrol launch file starts the complete navigation stack and automatically
visits the three map-frame poses configured in
`src/companion_robot_behaviors/config/patrol.yaml`:

```bash
ros2 launch companion_robot_behaviors patrol.launch.py
```

One patrol loop runs by default. Set a finite number of loops or repeat forever
with `loop_count:=0`:

```bash
ros2 launch companion_robot_behaviors patrol.launch.py loop_count:=2
ros2 launch companion_robot_behaviors patrol.launch.py loop_count:=0
```

Each waypoint is stored as three consecutive values: `x`, `y`, and yaw in
radians. Stop an active patrol with `Ctrl+C`. Do not publish manual WASD commands
while the patrol node is running.

To interrupt a continuous patrol and send the robot back to `(0, 0, 0)`, keep
the patrol launch running and call its service from a second sourced terminal:

```bash
ros2 service call /return_home std_srvs/srv/Trigger "{}"
```

The home pose can be changed through `home_pose` in `patrol.yaml`. The patrol
node cancels its current Nav2 goal before sending the home goal, so the two
commands do not compete.

### Docking Station and Poses

The south side of the Gazebo arena contains a reverse-entry docking station.
Its cyan floor marker is the Nav2 staging target, while the green floor marker
between the side guides shows the final dock target. A black-and-white ArUco
marker on the dock backboard provides the final camera reference. The map poses
are stored in
`src/companion_robot_behaviors/config/docking.yaml`:

| Pose | X | Y | Yaw | Purpose |
| --- | ---: | ---: | ---: | --- |
| Staging | 0.00 | -1.70 | 1.5708 | Nav2 approach and alignment point |
| Dock | 0.00 | -2.55 | 1.5708 | Final robot-centre position |

At both poses the robot faces north. Moving from staging to dock therefore uses
reverse motion.

### Start Automatic Docking

Start the simulation, Nav2, RViz, and the docking behavior together:

```bash
ros2 launch companion_robot_behaviors docking.launch.py
```

After Nav2 reports that navigation and localization are active, call the
docking service from a second sourced terminal:

```bash
ros2 service call /dock_robot std_srvs/srv/Trigger "{}"
```

Nav2 first drives the robot to the staging marker. The precision controller then
takes over `/cmd_vel`, acquires the ArUco marker through the rear camera, and
reverses slowly while correcting its lateral error. The saved map pose is kept
as an independent final-position cross-check, and the rear LiDAR remains the
emergency stop sensor. If the marker disappears, the robot stops immediately.
After a sustained loss it retreats to staging and retries within the configured
recovery limit. A disagreement between camera and map, an obstacle, or a
no-progress timeout reports `ERROR`. Do not run patrol or WASD control at the
same time.

The current docking state is published as a transient-local string, so a new
terminal can inspect the latest value at any time:

```bash
ros2 topic echo /docking_status std_msgs/msg/String \
  --qos-durability transient_local
```

The expected docking sequence is `IDLE`, `WAITING_FOR_NAV2`,
`NAVIGATING_TO_STAGING`, `ALIGNING_WITH_DOCK`, `ACQUIRING_DOCK_MARKER`,
`PRECISION_DOCKING`, `DOCKED`, and `CHARGING`. A sustained camera interruption
temporarily changes the docking state to `RECOVERING_DOCK_MARKER`.
When the battery reaches full capacity, the final state is `FULLY_CHARGED`.

The detector itself publishes `WAITING_FOR_CAMERA`, `SEARCHING`, `DETECTED`,
`OCCLUDED`, or `LOST` on `/dock_marker/status`. `OCCLUDED` means that a recent
detection was interrupted briefly; `LOST` means the timeout was exceeded.

### Inspect the Dock Camera in RViz

The detector publishes two RViz-compatible views. In RViz, choose **Add**, then:

1. Add an **Image** display and select `/dock_marker/debug_image` to see the
   camera image, detected outline, axes, and current detector status.
2. Add a **Marker** display and select `/dock_marker/visualization` to see the
   estimated marker pose in 3D.

The raw image remains available on `/rear_camera/image_raw`. Camera inspection
is optional; docking uses the same topics automatically when either
`docking.launch.py` or `energy_patrol.launch.py` is running.

### Simulate Low Battery and Automatic Docking

The docking launch also starts a battery simulator. It consumes energy while
the robot moves and charges while the robot is docked. To demonstrate the
low-battery path immediately, call:

```bash
ros2 service call /simulate_low_battery std_srvs/srv/Trigger "{}"
```

This sets the battery below its configured 25% threshold. The battery node then
calls `/dock_robot` automatically; no second manual docking request is needed.
Monitor the simulated battery with:

```bash
ros2 topic echo /battery_state sensor_msgs/msg/BatteryState
```

Charging is intentionally accelerated for demonstrations. Its rates, threshold,
and initial percentage are configurable in
`src/companion_robot_behaviors/config/battery.yaml`.

### Undock the Robot

When at least 50% battery is available, request undocking from another sourced
terminal:

```bash
ros2 service call /undock_robot std_srvs/srv/Trigger "{}"
```

The robot moves forward out of the station and stops at the staging pose. A
front LiDAR safety sector stops undocking if the exit is obstructed. The state
sequence is `CHARGING` or `FULLY_CHARGED`, then `UNDOCKING`, and finally `IDLE`.

### Run an Energy-Aware Patrol

This launch combines continuous waypoint patrol, docking, and battery
simulation in one state-aware workflow:

```bash
ros2 launch companion_robot_behaviors energy_patrol.launch.py
```

To demonstrate the complete cycle without waiting for normal discharge, use a
second sourced terminal:

```bash
ros2 service call /simulate_low_battery std_srvs/srv/Trigger "{}"
```

The mission manager is the single coordinator for this launch. The patrol
saves its current waypoint and pauses as soon as low-battery docking starts.
The robot docks and charges to 100%, automatically undocks to the staging pose,
and then retries the saved waypoint before continuing its patrol. The launch
repeats patrol loops indefinitely by default; pass `loop_count:=N` to use a
finite number of loops.

Mission state transitions are published on `/mission_status`. A compact status
snapshot, including subsystem states and recovery counters, is available from
a second sourced terminal:

```bash
ros2 service call /get_mission_status std_srvs/srv/Trigger "{}"
```

Expected states include `INITIALIZING`, `IDLE`, `PATROLLING`,
`RETURNING_HOME`, `DOCKING`, `CHARGING`, `FULLY_CHARGED`, `UNDOCKING`,
`RECOVERY`, and `ERROR`. If Nav2 requests motion without odometry progress for
15 seconds, the manager cancels and replans the current waypoint. Navigation
failures and failed low-battery docking cycles are retried up to the configured
limits in `src/companion_robot_behaviors/config/mission.yaml`.
The status snapshot also contains `dock_marker_status`, so camera perception can
be checked without opening a separate image window.

After inspecting and clearing the physical cause of a terminal `ERROR`, reset
the recovery counters and request another attempt with:

```bash
ros2 service call /recover_mission std_srvs/srv/Trigger "{}"
```

### Test a Moving Obstacle

The arena contains a purple pedestrian dummy that continuously crosses the
patrol route. Start autonomous navigation, waypoint patrol, or the complete
energy-aware patrol normally. The LiDAR marks this model in both Nav2
costmaps, allowing the local controller to slow down, stop, or select a clear
trajectory around it.

For a clear demonstration, run the energy-aware patrol and observe the local
costmap in RViz:

```bash
ros2 launch companion_robot_behaviors energy_patrol.launch.py
```

The Collision Monitor receives only Nav2 velocity commands. Its 360-degree
slowdown and emergency-stop envelopes protect the front, sides, and rear of
the robot. It first reduces the requested speed and publishes zero velocity if
an obstacle gets too close. Precision docking continues to use its separate
front/rear LiDAR protection. The moving pedestrian uses a physics-constrained
slide joint, so contact is resolved by Gazebo instead of passing through the
robot. It also reverses before entering the robot's clearance zone, modelling
a pedestrian that reacts instead of continuously pushing the robot.

The moving obstacle can be paused for comparison with the static arena:

```bash
ros2 service call /set_moving_obstacle_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

Change `false` to `true` to resume it. Launch the arena with
`moving_obstacle:=false` when the controller should remain disabled for the
whole session.

## Main ROS Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity command |
| `/scan` | `sensor_msgs/msg/LaserScan` | Simulated 2D LiDAR scan |
| `/rear_camera/image_raw` | `sensor_msgs/msg/Image` | Simulated rear RGB image |
| `/rear_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Rear-camera calibration |
| `/odom` | `nav_msgs/msg/Odometry` | Stable Gazebo pose odometry |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | Wheel-based odometry |
| `/joint_states` | `sensor_msgs/msg/JointState` | Wheel joint states |
| `/tf` | `tf2_msgs/msg/TFMessage` | Robot transform tree |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Map generated by SLAM |
| `/dock_robot` | `std_srvs/srv/Trigger` | Start automatic docking |
| `/undock_robot` | `std_srvs/srv/Trigger` | Leave the dock for the staging pose |
| `/simulate_low_battery` | `std_srvs/srv/Trigger` | Trigger a low-battery demonstration |
| `/get_mission_status` | `std_srvs/srv/Trigger` | Return one mission and subsystem snapshot |
| `/recover_mission` | `std_srvs/srv/Trigger` | Reset recovery limits and retry the failed mission |
| `/recover_patrol` | `std_srvs/srv/Trigger` | Cancel and replan the current patrol goal |
| `/set_moving_obstacle_enabled` | `std_srvs/srv/SetBool` | Pause or resume the moving obstacle |
| `/docking_status` | `std_msgs/msg/String` | Latest docking state |
| `/dock_marker/pose` | `geometry_msgs/msg/PoseStamped` | Marker pose relative to the rear camera |
| `/dock_marker/visible` | `std_msgs/msg/Bool` | Whether the current image contains the marker |
| `/dock_marker/confidence` | `std_msgs/msg/Float32` | Marker image-area confidence indicator |
| `/dock_marker/status` | `std_msgs/msg/String` | Camera and marker detection state |
| `/dock_marker/debug_image` | `sensor_msgs/msg/Image` | Annotated image for RViz diagnostics |
| `/patrol_status` | `std_msgs/msg/String` | Latest waypoint-patrol state |
| `/mission_status` | `std_msgs/msg/String` | Latest high-level mission state |
| `/mission_detail` | `std_msgs/msg/String` | Human-readable explanation of the mission state |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Simulated charge and power status |

The `/odom` topic currently comes from Gazebo's pose-based odometry publisher.
It provides a stable reference for learning SLAM. The separate `/wheel_odom`
topic remains available for later wheel-slip and encoder-odometry experiments.

## Roadmap

- Improve AMCL robustness with noisier odometry
- Tune costmaps and the local controller for tighter spaces
- Add mission-state visualization and diagnostics in RViz
- Test visual docking under stronger camera noise and partial occlusion
- Add person detection and human-following companion behaviour
- Transfer the software stack to physical hardware

## License

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE).
