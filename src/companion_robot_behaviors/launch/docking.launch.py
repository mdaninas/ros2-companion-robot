from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_rviz = LaunchConfiguration("open_rviz")
    docking_params = LaunchConfiguration("docking_params")
    battery_params = LaunchConfiguration("battery_params")
    auto_dock_enabled = LaunchConfiguration("auto_dock_enabled")
    auto_undock_when_full = LaunchConfiguration("auto_undock_when_full")

    behaviors_share = FindPackageShare("companion_robot_behaviors")
    navigation_share = FindPackageShare("companion_robot_navigation")

    default_docking_params = PathJoinSubstitution(
        [behaviors_share, "config", "docking.yaml"]
    )
    default_battery_params = PathJoinSubstitution(
        [behaviors_share, "config", "battery.yaml"]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [navigation_share, "launch", "navigation.launch.py"]
            )
        ),
        launch_arguments={"open_rviz": open_rviz}.items(),
    )

    docking = Node(
        package="companion_robot_behaviors",
        executable="auto_docking",
        name="docking_behavior",
        output="screen",
        parameters=[docking_params, {"use_sim_time": True}],
    )

    battery = Node(
        package="companion_robot_behaviors",
        executable="battery_simulator",
        name="battery_simulator",
        output="screen",
        parameters=[
            battery_params,
            {
                "use_sim_time": True,
                "auto_dock_enabled": ParameterValue(
                    auto_dock_enabled, value_type=bool
                ),
                "auto_undock_when_full": ParameterValue(
                    auto_undock_when_full, value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "open_rviz",
                default_value="true",
                description="Open RViz with the Nav2 default view.",
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
                "auto_dock_enabled",
                default_value="true",
                description="Automatically request docking at low battery.",
            ),
            DeclareLaunchArgument(
                "auto_undock_when_full",
                default_value="false",
                description="Automatically undock after charging reaches 100%.",
            ),
            navigation,
            docking,
            battery,
        ]
    )
