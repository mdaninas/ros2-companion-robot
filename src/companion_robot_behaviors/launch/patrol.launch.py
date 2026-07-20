from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_rviz = LaunchConfiguration("open_rviz")
    loop_count = LaunchConfiguration("loop_count")
    patrol_params = LaunchConfiguration("patrol_params")

    behaviors_share = FindPackageShare("companion_robot_behaviors")
    navigation_share = FindPackageShare("companion_robot_navigation")

    default_patrol_params = PathJoinSubstitution(
        [behaviors_share, "config", "patrol.yaml"]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [navigation_share, "launch", "navigation.launch.py"]
            )
        ),
        launch_arguments={"open_rviz": open_rviz}.items(),
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "open_rviz",
                default_value="true",
                description="Open RViz with the Nav2 default view.",
            ),
            DeclareLaunchArgument(
                "loop_count",
                default_value="1",
                description="Number of patrol loops; zero repeats forever.",
            ),
            DeclareLaunchArgument(
                "patrol_params",
                default_value=default_patrol_params,
                description="Path to the waypoint-patrol parameter file.",
            ),
            navigation,
            patrol,
        ]
    )
