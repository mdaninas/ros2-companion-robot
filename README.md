# ROS 2 Companion Robot

A mobile companion-rover simulation built with ROS 2 Jazzy and Gazebo
Harmonic. The project covers the robot model, differential-drive motion, 2D
LiDAR, front and rear RGB cameras, manual control, noisy simulation odometry,
SLAM, and a saved occupancy map.

The current milestone is robust autonomous localization, navigation, waypoint
patrol, energy-aware docking, mission-level recovery, multi-person identity
tracking, predictive social human following, and live RViz diagnostics with
Nav2.

## Features

- Parametric robot model written in URDF/Xacro
- Differential-drive physics in Gazebo Sim
- Two drive wheels and front/rear caster support
- Simulated 360-degree 2D LiDAR
- Simulated rear RGB camera with ROS image and calibration topics
- Simulated front RGB camera for person detection
- Safe terminal teleoperation using `W`, `A`, `S`, and `D`
- Gazebo pose odometry with modest measurement noise on `/odom`
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
- Two moving pedestrian obstacles detected through LiDAR and Nav2 costmaps
- Direction-aware LiDAR slowdown and emergency-stop protection
- Central mission state manager with automatic patrol and docking recovery
- Velocity smoothing for less abrupt autonomous motion under WSL
- Two-stage AMCL recovery using a sensor refresh, then global relocalization
- Costmap clearing before patrol replanning
- Live autonomy health, patrol route, costmaps, and sensor status in RViz
- Camera-and-LiDAR person detection using rendered sensor data
- Identity-locked predictive human following with map-frame motion estimation
- Adaptive social distance, path-aware trailing, approach yielding, continuous
  front-camera alignment, safe retreat, and lost-target motion prediction

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
| Nav2 localization and autonomous navigation | Robust simulation implementation |
| Multi-waypoint patrol | Validated in simulation |
| Return to home | Initial implementation |
| Docking station and docking poses | Complete |
| Camera-guided precision auto-docking | Complete in simulation |
| Marker-loss safety and recovery | Validated in simulation |
| Automatic undocking | Validated in simulation |
| Battery and charging simulation | Validated in simulation |
| Low-battery docking trigger | Validated in simulation |
| Energy-aware patrol pause and resume | Validated in simulation |
| Dynamic obstacle avoidance | Validated in simulation |
| Mission state and autonomous recovery | Validated in simulation |
| Autonomy diagnostics and RViz health overlay | Complete |
| Front-camera person detection | Validated in simulation |
| Predictive human following and safe distance control | Implemented; manual validation pending |
| Adaptive distance and social approach yielding | Implemented; manual validation pending |
| Multi-person identity selection and lost-target recovery | Validated in simulation |
| Physical robot deployment | Planned |

## Project Structure

```text
.
|-- src/
|   |-- companion_robot_behaviors/
|   |   |-- config/        # Patrol, docking, following, battery, and mission
|   |   |-- launch/        # Autonomous behavior launch files
|   |   `-- scripts/       # Patrol, docking, battery, and mission nodes
|   |-- companion_robot_description/
|   |   |-- launch/        # Standalone robot visualization
|   |   |-- rviz/          # RViz model configuration
|   |   `-- urdf/          # Parametric robot model
|   |-- companion_robot_gazebo/
|   |   |-- config/        # SLAM and moving-person parameters
|   |   |-- launch/        # Simulation and mapping launch files
|   |   |-- maps/          # Saved occupancy maps
|   |   |-- rviz/          # Simulation and mapping views
|   |   |-- scripts/       # WASD and moving-person controllers
|   |   `-- worlds/        # Gazebo arena
|   |-- companion_robot_perception/
|   |   |-- config/        # Dock-marker and person-detector parameters
|   |   `-- scripts/       # Rear-marker and front-person perception nodes
|   `-- companion_robot_navigation/
|       |-- config/        # AMCL, costmap, planner, and controller parameters
|       |-- launch/        # Autonomous-navigation launch file
|       `-- rviz/          # Navigation, health, route, and costmap view
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

### Understand the Terminal Layout

Run only one main launch file at a time. A launch such as
`human_following.launch.py` already starts Gazebo, the robot, localization,
Nav2, perception, behavior nodes, and RViz. Starting `arena.launch.py` or
`navigation.launch.py` beside it creates a second competing simulation.

Most demonstrations need two terminals:

| Terminal | Purpose |
| --- | --- |
| Terminal 1 | Keep one complete launch running |
| Terminal 2 | Send a service request or inspect status |

The second terminal is not another robot process. It is only a temporary
control and diagnostics console. Source ROS 2 and the workspace in every new
terminal:

```bash
cd /path/to/companion_robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

After editing source files, rebuild the affected packages and restart Terminal
1. A process that is already running does not reload changed Python or launch
files automatically.

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

The supplied navigation RViz view now shows the saved map, local/global
costmaps, AMCL particles, LiDAR, planned path, patrol route, and a coloured
health indicator above the robot. Green means healthy, yellow means attention
or recovery, and red means a terminal error. The dock camera debug display is
included but disabled by default; enable **Dock Camera Debug** only when the
rear image needs inspection.

Mission state transitions are published on `/mission_status`. A compact status
snapshot, including subsystem states and recovery counters, is available from
a second sourced terminal:

```bash
ros2 service call /get_mission_status std_srvs/srv/Trigger "{}"
```

For a health-focused snapshot covering mission state, battery, odometry,
LiDAR, AMCL uncertainty, navigation, and docking, call:

```bash
ros2 service call /get_autonomy_health std_srvs/srv/Trigger "{}"
```

Expected states include `INITIALIZING`, `IDLE`, `PATROLLING`,
`RETURNING_HOME`, `DOCKING`, `CHARGING`, `FULLY_CHARGED`, `UNDOCKING`,
`RECOVERY`, and `ERROR`. If Nav2 requests motion without odometry progress for
15 seconds, the manager cancels and replans the current waypoint. Navigation
failures and failed low-battery docking cycles are retried up to the configured
limits in `src/companion_robot_behaviors/config/mission.yaml`.
The status snapshot also contains `dock_marker_status`, so camera perception can
be checked without opening a separate image window.

When AMCL becomes stale or excessively uncertain, recovery happens in two
steps. The mission manager first requests a no-motion laser update. If that is
not enough, it requests global relocalization and replans the current patrol
goal. A failed Nav2 goal also clears both costmaps before retrying, preventing a
short-lived moving obstacle from poisoning every later plan.

After inspecting and clearing the physical cause of a terminal `ERROR`, reset
the recovery counters and request another attempt with:

```bash
ros2 service call /recover_mission std_srvs/srv/Trigger "{}"
```

### Test a Moving Obstacle

The arena contains two physical pedestrian dummies with different clothing
colours and intersecting waypoint loops. They walk diagonally, change speed,
and pause at different points. Start autonomous navigation, waypoint patrol,
or the complete energy-aware patrol normally. LiDAR marks both models in the
Nav2 costmaps, allowing the local controller to slow down, stop, or select a
clear trajectory around them.

For a clear demonstration, run the energy-aware patrol and observe the local
costmap in RViz:

```bash
ros2 launch companion_robot_behaviors energy_patrol.launch.py
```

The directional safety filter receives only smoothed Nav2 velocity commands.
It checks the LiDAR sector in the requested travel direction, slows down near
an obstacle, and publishes zero velocity if that path is unsafe. A person in
front therefore does not prevent a safe reverse command, while an obstacle
behind still does. Stale LiDAR or velocity input also produces a safe stop.
Precision docking continues to use its separate front/rear LiDAR protection.
Each moving pedestrian uses a physics-constrained two-axis stage, so contact
is resolved by Gazebo instead of passing through the robot. If the robot enters
the pedestrian's clearance zone, the pedestrian continues only along a
configured route segment that increases separation. If no route segment is
safe, it waits. This prevents both pushing and a permanent mutual-waiting
deadlock without adding an artificial escape trajectory.

Each person can be paused independently for occlusion and recovery tests:

```bash
ros2 service call /set_person_alpha_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /set_person_beta_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

Change `false` to `true` to resume it. Launch the arena with
`moving_obstacle:=false` when the controller should remain disabled for the
whole session.

### Follow the Simulated Person

The human-following launch starts the arena, saved-map localization, Nav2,
front-camera person detector, following controller, and RViz together:

```bash
ros2 launch companion_robot_behaviors human_following.launch.py
```

The simulated people wear purple (`person_alpha`) and blue (`person_beta`).
The detector maintains a separate colour-and-range track for each identity.
Camera bearing is associated with current LiDAR returns to estimate metric
target poses; the behavior does not read either person's ground-truth Gazebo
pose. Selecting one identity locks the follower to that person even when the
two routes cross.

The robot follows at a base 0.70 metre surface distance and treats 0.58 metres
as a hard minimum. A retreat remains active until 0.66 metres so sensor noise
cannot immediately cancel it. During retreat, a direct bounded controller
reverses while steering the fixed front camera toward the person instead of
letting the global planner choose the turn direction. The robot holds that
heading while nearby. An exclusive command mux prevents Nav2 from overwriting
direct facing or retreat commands. The selected command still passes through
velocity smoothing and direction-aware collision protection.

The follower estimates the selected person's filtered velocity in the `map`
frame. This avoids treating the robot's own translation or rotation as human
motion. After four consistent samples, it predicts a bounded future target,
blends the following goal toward the person's direction of travel, and trails
behind that direction rather than cutting across the person's path. Faster
motion increases the requested gap gradually from 0.70 up to 0.90 metres.
Prediction is disabled automatically while the estimate is immature, stale,
or implausibly fast.

When the selected person walks toward the robot within the social-yield zone,
the robot cancels translation and keeps its front camera facing the person.
The hard minimum-distance retreat still has priority if the person continues
approaching. Hysteresis prevents rapid switching at the yield boundary.
If the selected target leaves the front camera, the current goal is cancelled.
The robot first faces the person's bounded motion-predicted position, then
performs a bounded left/right sweep. It resumes only when the same selected
identity is detected again. After 15 seconds without reacquisition it briefly
enters `TARGET_LOST`, then periodically performs a complete sweep. This keeps
the robot stationary between attempts without permanently giving up or
switching to the other person.

Inspect the current state from a second sourced terminal:

```bash
ros2 service call /get_human_following_status std_srvs/srv/Trigger "{}"
```

Expected runtime states are `SEARCHING`, `FOLLOWING`,
`PREDICTIVE_FOLLOWING`, `HOLDING_DISTANCE`, `SOCIAL_YIELDING`,
`TURNING_TO_PERSON`, `RETREATING`, `SEARCHING_LAST_SEEN`, `SEARCHING_SWEEP`,
`SEARCHING_REACQUIRE`, and `TARGET_LOST`. The returned JSON includes
`selected_identity`, `target_visible`, `target_distance`, `target_speed`,
`motion_confidence`, `predicted_target_map`, `effective_follow_distance`,
`closing_speed`, `social_mode`, `search_phase`, and the last known map
position.

List both tracks and see which identity is selected:

```bash
ros2 service call /get_person_targets std_srvs/srv/Trigger "{}"
```

Change the target while the simulation is running:

```bash
ros2 param set /person_detector selected_identity person_beta
```

Use `person_alpha` to switch back, or cycle through configured identities:

```bash
ros2 service call /cycle_person_target std_srvs/srv/Trigger "{}"
```

For a recovery test, let the selected person cross an obstacle or the camera
edge. The normal sequence is:

1. `SEARCHING_LAST_SEEN` while the robot faces the last known map position.
2. `SEARCHING_SWEEP` during the initial bounded search.
3. A brief safe `TARGET_LOST` pause if the first search expires.
4. `SEARCHING_REACQUIRE` during each periodic complete sweep.
5. `TURNING_TO_PERSON`, `FOLLOWING`, or `HOLDING_DISTANCE` after the same
   identity is found again.

The selected identity must remain unchanged throughout this sequence. The
pause services stop route motion but do not hide a person from the sensors.
Human following can also be disabled and enabled without restarting the
simulation:

```bash
ros2 service call /stop_human_following std_srvs/srv/Trigger "{}"
ros2 service call /start_human_following std_srvs/srv/Trigger "{}"
```

RViz displays all fresh identity tracks, highlights the selected one, and
shows the current following goal. The purple cylinder is the measured person,
the green sphere is the predicted position, the green arrow is estimated
motion, the orange sphere is the Nav2 goal, and the translucent yellow circle
is the current social-distance radius. Enable the disabled
`/person_detection/debug_image` Image display to inspect labelled camera
boxes and fused ranges.

### Validate Human Following Manually

Use two terminals. Start the complete stack in Terminal 1:

```bash
ros2 launch companion_robot_behaviors human_following.launch.py
```

In Terminal 2, select the other person and inspect the follower:

```bash
ros2 service call /cycle_person_target std_srvs/srv/Trigger "{}"
ros2 service call /get_human_following_status std_srvs/srv/Trigger "{}"
ros2 service call /get_person_targets std_srvs/srv/Trigger "{}"
```

After one cycle from the default selection, `selected_identity` should be
`person_beta`, the blue person. Verify all of the following:

- The robot turns its front camera toward the selected person.
- A moving person produces a short stable green velocity arrow and a green
  predicted point slightly ahead, never more than 0.35 metres away.
- The status changes to `PREDICTIVE_FOLLOWING` after motion becomes reliable.
- The orange Nav2 goal trails the direction of travel instead of cutting
  directly across the person's route.
- It holds approximately 0.70 metres from a stationary person and may
  gradually increase the gap up to 0.90 metres while the person moves.
- When the person approaches within the yield zone, the robot enters
  `SOCIAL_YIELDING`, stops translating, and continues facing the person.
- It enters `RETREATING` when the distance drops below 0.58 metres.
- Losing the target causes a safe search rather than uncontrolled translation.
- The first lost-target turn uses the bounded predicted direction.
- A later reacquisition still selects `person_beta`, even if `person_alpha` is
  also visible.
- The LiDAR scan remains aligned with the arena and contains no persistent
  rays through the robot body.

The implementation is not behaving correctly if the green prediction jumps
around while the person is still, exceeds the bounded lead, points opposite
the person's sustained travel, causes the robot to cut in front of the person,
changes identity without a request, oscillates rapidly between yield and
follow, or allows translation to continue inside the hard minimum distance.

### Troubleshooting

**A frame does not exist in RViz**

Short frame warnings are normal while the simulation is starting. If the
warning remains after Nav2 becomes active, stop the launch with `Ctrl+C`, make
sure no other companion-robot launch is still running, source the workspace,
and start one launch again. Do not run the arena, navigation, and behavior
launch files simultaneously.

**A recent source change does not appear**

Rebuild the changed package, source the workspace again, and restart the main
launch. For the current person-following stack:

```bash
colcon build --symlink-install --packages-select \
  companion_robot_description companion_robot_gazebo \
  companion_robot_navigation companion_robot_perception \
  companion_robot_behaviors
source install/setup.bash
```

**The robot is stationary**

First inspect `/get_human_following_status` or `/get_mission_status`. A
stationary robot can be correct: `HOLDING_DISTANCE`, `TARGET_LOST`,
`CHARGING`, and collision-safety stop states intentionally command zero
velocity. Treat `ERROR`, stale sensors, or a state that never changes as a
fault requiring investigation.

**Gazebo opens but `gz` is not found in a fresh terminal**

Source ROS 2 first:

```bash
source /opt/ros/jazzy/setup.bash
```

## Main ROS Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity command |
| `/scan` | `sensor_msgs/msg/LaserScan` | Simulated 2D LiDAR scan |
| `/rear_camera/image_raw` | `sensor_msgs/msg/Image` | Simulated rear RGB image |
| `/rear_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Rear-camera calibration |
| `/front_camera/image_raw` | `sensor_msgs/msg/Image` | Simulated front RGB image |
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
| `/get_autonomy_health` | `std_srvs/srv/Trigger` | Return combined mission, sensor, and localization health |
| `/recover_mission` | `std_srvs/srv/Trigger` | Reset recovery limits and retry the failed mission |
| `/recover_patrol` | `std_srvs/srv/Trigger` | Cancel and replan the current patrol goal |
| `/set_person_alpha_enabled` | `std_srvs/srv/SetBool` | Pause or resume `person_alpha` |
| `/set_person_beta_enabled` | `std_srvs/srv/SetBool` | Pause or resume `person_beta` |
| `/docking_status` | `std_msgs/msg/String` | Latest docking state |
| `/dock_marker/pose` | `geometry_msgs/msg/PoseStamped` | Marker pose relative to the rear camera |
| `/dock_marker/visible` | `std_msgs/msg/Bool` | Whether the current image contains the marker |
| `/dock_marker/confidence` | `std_msgs/msg/Float32` | Marker image-area confidence indicator |
| `/dock_marker/status` | `std_msgs/msg/String` | Camera and marker detection state |
| `/dock_marker/debug_image` | `sensor_msgs/msg/Image` | Annotated image for RViz diagnostics |
| `/patrol_status` | `std_msgs/msg/String` | Latest waypoint-patrol state |
| `/patrol/visualization` | `visualization_msgs/msg/MarkerArray` | Waypoints, route, active target, and home marker for RViz |
| `/mission_status` | `std_msgs/msg/String` | Latest high-level mission state |
| `/mission_detail` | `std_msgs/msg/String` | Human-readable explanation of the mission state |
| `/autonomy/health` | `std_msgs/msg/String` | Combined `OK`, `WARN`, `ERROR`, or `STALE` health state |
| `/autonomy/visualization` | `visualization_msgs/msg/MarkerArray` | Coloured status indicator and mission text in RViz |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Detailed subsystem health for ROS diagnostic tools |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Simulated charge and power status |
| `/person_detection/pose` | `geometry_msgs/msg/PoseStamped` | Camera-and-LiDAR target pose in the robot frame |
| `/person_detection/status` | `std_msgs/msg/String` | Latest detector state |
| `/person_detection/selected_identity` | `std_msgs/msg/String` | Identity currently locked for following |
| `/person_detection/tracks` | `std_msgs/msg/String` | JSON snapshot of all configured identity tracks |
| `/person_detection/tracks_visualization` | `visualization_msgs/msg/MarkerArray` | RViz markers and labels for fresh person tracks |
| `/person_detection/debug_image` | `sensor_msgs/msg/Image` | Annotated front-camera image |
| `/get_person_targets` | `std_srvs/srv/Trigger` | Return selected identity and all current tracks |
| `/cycle_person_target` | `std_srvs/srv/Trigger` | Select the next configured person identity |
| `/human_following/status` | `std_msgs/msg/String` | Latest human-following state |
| `/human_following/predicted_target` | `geometry_msgs/msg/PoseStamped` | Bounded future person position in the map frame |
| `/human_following/target_velocity` | `geometry_msgs/msg/TwistStamped` | Filtered selected-person velocity in the map frame |
| `/human_following/effective_distance` | `std_msgs/msg/Float32` | Current speed-adaptive following distance |
| `/human_following/visualization` | `visualization_msgs/msg/MarkerArray` | Measured target, prediction, velocity, social radius, and goal |
| `/get_human_following_status` | `std_srvs/srv/Trigger` | Return target range and following state |
| `/start_human_following` | `std_srvs/srv/Trigger` | Enable human following |
| `/stop_human_following` | `std_srvs/srv/Trigger` | Cancel the goal and stop following |

The `/odom` topic comes from Gazebo's pose-based odometry publisher with modest
Gaussian measurement noise. This keeps the simulation approachable while
requiring AMCL to perform real corrections. The separate `/wheel_odom` topic
remains available for later wheel-slip and encoder-odometry experiments.

## Roadmap

- Validate camera-guided docking under changing light and stronger occlusion
- Validate predictive following and social navigation across varied routes
- Replace pose odometry with fused wheel encoder and IMU odometry
- Transfer the software stack to physical hardware

## License

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE).
