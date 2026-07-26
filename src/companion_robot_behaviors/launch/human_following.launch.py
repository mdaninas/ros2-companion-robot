from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_rviz = LaunchConfiguration("open_rviz")
    headless = LaunchConfiguration("headless")
    detector_params = LaunchConfiguration("detector_params")
    following_params = LaunchConfiguration("following_params")

    navigation_share = FindPackageShare("companion_robot_navigation")
    perception_share = FindPackageShare("companion_robot_perception")
    behaviors_share = FindPackageShare("companion_robot_behaviors")

    default_detector_params = PathJoinSubstitution(
        [perception_share, "config", "person_detector.yaml"]
    )
    default_following_params = PathJoinSubstitution(
        [behaviors_share, "config", "following.yaml"]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [navigation_share, "launch", "navigation.launch.py"]
            )
        ),
        launch_arguments={
            "open_rviz": open_rviz,
            "headless": headless,
        }.items(),
    )

    person_detector = Node(
        package="companion_robot_perception",
        executable="person_detector",
        name="person_detector",
        output="screen",
        parameters=[detector_params, {"use_sim_time": True}],
    )

    human_follower = Node(
        package="companion_robot_behaviors",
        executable="human_follower",
        name="human_follower",
        output="screen",
        parameters=[following_params, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "open_rviz",
                default_value="true",
                description="Open RViz with human-following visualizations.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without its 3D window.",
            ),
            DeclareLaunchArgument(
                "detector_params",
                default_value=default_detector_params,
                description="Path to person-detector parameters.",
            ),
            DeclareLaunchArgument(
                "following_params",
                default_value=default_following_params,
                description="Path to human-following parameters.",
            ),
            navigation,
            person_detector,
            human_follower,
        ]
    )
