from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_rviz = LaunchConfiguration("open_rviz")
    docking_params = LaunchConfiguration("docking_params")
    battery_params = LaunchConfiguration("battery_params")

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
        parameters=[battery_params, {"use_sim_time": True}],
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
            navigation,
            docking,
            battery,
        ]
    )
