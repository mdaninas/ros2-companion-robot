from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_share = FindPackageShare("companion_robot_gazebo")
    slam_share = FindPackageShare("slam_toolbox")

    slam_params = PathJoinSubstitution(
        [gazebo_share, "config", "slam.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [gazebo_share, "rviz", "mapping.rviz"]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_share, "launch", "arena.launch.py"])
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [slam_share, "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    # Let Gazebo, the robot, and their TF topics start before SLAM activates.
    delayed_slam = TimerAction(period=3.0, actions=[slam])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="mapping_rviz",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([simulation, delayed_slam, rviz])
