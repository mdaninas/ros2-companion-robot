from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Keep this name distinct from arena.launch.py's use_rviz argument.
    # Launch configurations share a context across included launch files.
    open_rviz = LaunchConfiguration("open_rviz")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    gazebo_share = FindPackageShare("companion_robot_gazebo")
    navigation_share = FindPackageShare("companion_robot_navigation")
    nav2_bringup_share = FindPackageShare("nav2_bringup")

    default_map = PathJoinSubstitution(
        [gazebo_share, "maps", "companion_arena.yaml"]
    )
    default_params = PathJoinSubstitution(
        [navigation_share, "config", "nav2_params.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [nav2_bringup_share, "rviz", "nav2_default_view.rviz"]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_share, "launch", "arena.launch.py"])
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    common_parameters = [params_file, {"use_sim_time": True}]
    autostart_value = ParameterValue(autostart, value_type=bool)

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "yaml_filename": map_yaml,
            },
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=common_parameters,
    )

    localization_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": autostart_value,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=common_parameters,
        remappings=[("cmd_vel", "/cmd_vel_nav")],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common_parameters,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=common_parameters,
        remappings=[("cmd_vel", "/cmd_vel_nav")],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=common_parameters,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=common_parameters,
    )

    navigation_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": autostart_value,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "collision_monitor",
                ],
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(open_rviz),
    )

    delayed_localization = TimerAction(
        period=3.0,
        actions=[map_server, amcl, localization_manager],
    )

    delayed_navigation = TimerAction(
        period=5.0,
        actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            collision_monitor,
            navigation_manager,
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
                "autostart",
                default_value="true",
                description="Automatically activate localization and navigation nodes.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Absolute path to the occupancy-map YAML file.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Absolute path to the Nav2 parameter file.",
            ),
            simulation,
            delayed_localization,
            delayed_navigation,
            rviz,
        ]
    )
