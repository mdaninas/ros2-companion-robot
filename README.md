# ROS 2 Companion Robot

A mobile companion-rover simulation built with ROS 2 Jazzy and Gazebo
Harmonic. The project currently covers the robot model, differential-drive
motion, 2D LiDAR, manual control, odometry, SLAM, and a saved occupancy map.

The next milestone is autonomous localization and navigation with Nav2.

## Features

- Parametric robot model written in URDF/Xacro
- Differential-drive physics in Gazebo Sim
- Two drive wheels and front/rear caster support
- Simulated 360-degree 2D LiDAR
- Safe terminal teleoperation using `W`, `A`, `S`, and `D`
- Gazebo pose odometry on `/odom`
- Encoder-style wheel odometry on `/wheel_odom` for comparison
- Online mapping with SLAM Toolbox
- RViz configurations for the robot, simulation, and mapping
- Reusable 8 x 6 metre arena with three static obstacles
- Saved map ready for the upcoming Nav2 milestone

## Current Status

| Capability | Status |
| --- | --- |
| Robot model and RViz visualization | Complete |
| Gazebo arena and robot spawning | Complete |
| Differential-drive control | Complete |
| LiDAR and odometry bridge | Complete |
| SLAM mapping and map export | Complete |
| Nav2 localization and autonomous navigation | Next milestone |
| Physical robot deployment | Planned |

## Project Structure

```text
.
|-- src/
|   |-- companion_robot_description/
|   |   |-- launch/        # Standalone robot visualization
|   |   |-- rviz/          # RViz model configuration
|   |   `-- urdf/          # Parametric robot model
|   `-- companion_robot_gazebo/
|       |-- config/        # SLAM Toolbox parameters
|       |-- launch/        # Simulation and mapping launch files
|       |-- maps/          # Saved occupancy maps
|       |-- rviz/          # Simulation and mapping views
|       |-- scripts/       # WASD teleoperation node
|       `-- worlds/        # Gazebo arena
|-- .gitignore
|-- LICENSE
`-- README.md
```

## Requirements

- Ubuntu 24.04, either native or through WSL 2 with WSLg
- ROS 2 Jazzy
- Gazebo Harmonic and the ROS-Gazebo integration packages
- `colcon` and `rosdep`

All ROS package dependencies are declared in the two `package.xml` files.

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

## Main ROS Interfaces

| Topic | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity command |
| `/scan` | `sensor_msgs/msg/LaserScan` | Simulated 2D LiDAR scan |
| `/odom` | `nav_msgs/msg/Odometry` | Stable Gazebo pose odometry |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | Wheel-based odometry |
| `/joint_states` | `sensor_msgs/msg/JointState` | Wheel joint states |
| `/tf` | `tf2_msgs/msg/TFMessage` | Robot transform tree |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Map generated by SLAM |

The `/odom` topic currently comes from Gazebo's pose-based odometry publisher.
It provides a stable reference for learning SLAM. The separate `/wheel_odom`
topic remains available for later wheel-slip and encoder-odometry experiments.

## Roadmap

- Load the saved map with Nav2
- Localize the robot using AMCL
- Send navigation goals from RViz
- Add local and global costmaps for obstacle avoidance
- Add companion behaviours such as patrol and return-to-dock
- Add camera perception
- Transfer the software stack to physical hardware

## License

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE).
