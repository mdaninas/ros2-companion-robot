from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_rviz = LaunchConfiguration("open_rviz")
    headless = LaunchConfiguration("headless")
    loop_count = LaunchConfiguration("loop_count")
    patrol_params = LaunchConfiguration("patrol_params")
    docking_params = LaunchConfiguration("docking_params")
    battery_params = LaunchConfiguration("battery_params")
    marker_params = LaunchConfiguration("marker_params")
    mission_params = LaunchConfiguration("mission_params")

    behaviors_share = FindPackageShare("companion_robot_behaviors")
    default_patrol_params = PathJoinSubstitution(
        [behaviors_share, "config", "patrol.yaml"]
    )
    default_docking_params = PathJoinSubstitution(
        [behaviors_share, "config", "docking.yaml"]
    )
    default_battery_params = PathJoinSubstitution(
        [behaviors_share, "config", "battery.yaml"]
    )
    default_mission_params = PathJoinSubstitution(
        [behaviors_share, "config", "mission.yaml"]
    )
    perception_share = FindPackageShare("companion_robot_perception")
    default_marker_params = PathJoinSubstitution(
        [perception_share, "config", "dock_marker.yaml"]
    )

    docking_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [behaviors_share, "launch", "docking.launch.py"]
            )
        ),
        launch_arguments={
            "open_rviz": open_rviz,
            "headless": headless,
            "docking_params": docking_params,
            "battery_params": battery_params,
            "marker_params": marker_params,
            "auto_dock_enabled": "false",
            "auto_undock_when_full": "false",
        }.items(),
    )

    patrol = Node(
        package="companion_robot_behaviors",
        executable="waypoint_patrol",
        name="waypoint_patrol",
        output="screen",
        parameters=[
            patrol_params,
            {
                "use_sim_time": True,
                "loop_count": ParameterValue(loop_count, value_type=int),
            },
        ],
    )

    mission_manager = Node(
        package="companion_robot_behaviors",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[mission_params, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "open_rviz",
                default_value="true",
                description="Open RViz with the Nav2 default view.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without its 3D window.",
            ),
            DeclareLaunchArgument(
                "loop_count",
                default_value="0",
                description="Number of patrol loops; zero repeats forever.",
            ),
            DeclareLaunchArgument(
                "patrol_params",
                default_value=default_patrol_params,
                description="Path to the waypoint-patrol parameter file.",
            ),
            DeclareLaunchArgument(
                "docking_params",
                default_value=default_docking_params,
                description="Path to the auto-docking parameter file.",
            ),
            DeclareLaunchArgument(
                "battery_params",
                default_value=default_battery_params,
                description="Path to the battery-simulation parameter file.",
            ),
            DeclareLaunchArgument(
                "mission_params",
                default_value=default_mission_params,
                description="Path to the mission-manager parameter file.",
            ),
            DeclareLaunchArgument(
                "marker_params",
                default_value=default_marker_params,
                description="Path to the dock-marker detector parameters.",
            ),
            docking_stack,
            patrol,
            mission_manager,
        ]
    )
