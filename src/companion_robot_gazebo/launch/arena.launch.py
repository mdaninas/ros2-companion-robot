from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    moving_obstacle = LaunchConfiguration("moving_obstacle")
    moving_obstacle_params = LaunchConfiguration("moving_obstacle_params")

    description_share = FindPackageShare("companion_robot_description")
    gazebo_share = FindPackageShare("companion_robot_gazebo")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "companion_robot.urdf.xacro"]
    )
    world_file = PathJoinSubstitution(
        [gazebo_share, "worlds", "companion_arena.sdf"]
    )
    rviz_config = PathJoinSubstitution(
        [gazebo_share, "rviz", "simulation.rviz"]
    )
    default_moving_obstacle_params = PathJoinSubstitution(
        [gazebo_share, "config", "moving_obstacle.yaml"]
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), ' "', xacro_file, '"']),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        # Quote the world path so workspaces containing spaces are supported.
        launch_arguments={"gz_args": ["-r -v 3 \"", world_file, "\""]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            # Gazebo -> ROS simulation time.
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # ROS -> Gazebo velocity commands.
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # ROS -> Gazebo velocity for the pedestrian's physical slide joint.
            (
                "/model/moving_pedestrian/joint/"
                "pedestrian_slide_joint/cmd_vel"
                "@std_msgs/msg/Float64]gz.msgs.Double"
            ),
            # Gazebo -> ROS stable pose odometry, wheel odometry, and TF.
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # Gazebo -> ROS 360-degree LiDAR scan.
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # Gazebo -> ROS wheel joint positions.
            (
                "/world/companion_arena/model/companion_robot/joint_state"
                "@sensor_msgs/msg/JointState[gz.msgs.Model"
            ),
        ],
        remappings=[
            (
                "/world/companion_arena/model/companion_robot/joint_state",
                "/joint_states",
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="simulation_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    moving_obstacle_controller = Node(
        package="companion_robot_gazebo",
        executable="moving_obstacle_controller",
        name="moving_obstacle_controller",
        output="screen",
        parameters=[moving_obstacle_params, {"use_sim_time": True}],
        condition=IfCondition(moving_obstacle),
    )

    # Give the Gazebo server a moment to advertise its entity-creation service.
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_companion_robot",
                output="screen",
                arguments=[
                    "-world",
                    "companion_arena",
                    "-topic",
                    "/robot_description",
                    "-name",
                    "companion_robot",
                    "-allow_renaming",
                    "true",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.08",
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Open RViz with the robot and LiDAR scan displays.",
            ),
            DeclareLaunchArgument(
                "moving_obstacle",
                default_value="true",
                description="Move the pedestrian dummy across the patrol route.",
            ),
            DeclareLaunchArgument(
                "moving_obstacle_params",
                default_value=default_moving_obstacle_params,
                description="Moving-obstacle controller parameter file.",
            ),
            gazebo,
            robot_state_publisher,
            bridge,
            moving_obstacle_controller,
            rviz,
            spawn_robot,
        ]
    )
