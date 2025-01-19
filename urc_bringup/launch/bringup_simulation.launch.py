import os
from xacro import process_file
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    # pkg_urc_gazebo = get_package_share_directory("urc_gazebo")
    pkg_urc_bringup = get_package_share_directory("urc_bringup")
    pkg_path_planning = get_package_share_directory("path_planning")
    pkg_trajectory_following = get_package_share_directory("trajectory_following")
    pkg_urc_localization = get_package_share_directory("urc_localization")

    controller_config_file_dir = os.path.join(
        pkg_urc_bringup, "config", "ros2_control_walli.yaml"
    )

    # world_path = os.path.join(pkg_urc_gazebo, "urdf/worlds/urc_world.world")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    xacro_file = os.path.join(
        get_package_share_directory("urc_hw_description"), "urdf/walli.xacro"
    )
    assert os.path.exists(xacro_file), "urdf path doesnt exist in " + str(xacro_file)
    robot_description_config = process_file(
        xacro_file, mappings={"use_simulation": "true"}
    )
    robot_desc = robot_description_config.toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        ),
        launch_arguments={"use_sim_time": "true"}.items(),  # "world": world_path
    )

    enable_color = SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1")

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "walli",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "5",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "0",
            "-topic",
            "robot_description",
        ],
    )

    load_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_desc},
        ],
        output="screen",
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["-p", controller_config_file_dir, "joint_state_broadcaster"],
    )

    # load_arm_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["-p", controller_config_file_dir, "arm_controller"],
    # )

    # load_gripper_controller_left = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["-p", controller_config_file_dir,
    #                "gripper_controller_left"],
    # )

    # load_gripper_controller_right = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["-p", controller_config_file_dir,
    #                "gripper_controller_right"],
    # )

    load_drivetrain_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["-p", controller_config_file_dir, "rover_drivetrain_controller"],
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("urc_bringup"), "/launch/teleop.launch.py"]
        )
    )

    twist_mux_node = Node(
        package="urc_platform",
        executable="urc_platform_TwistMux",
        name="twist_mux",
    )

    sim_gps_handler_node = Node(
        package="urc_platform",
        executable="urc_platform_SimGpsHandler",
        name="sim_gps_handler",
    )

    bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_urc_bringup, "launch", "bt.launch.py")
        )
    )

    path_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path_planning, "launch", "planning.launch.py")
        )
    )

    trajectory_following_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_trajectory_following,
                "launch",
                "trajectory_following.launch.py",
            )
        )
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_urc_localization, "launch", "ekf.launch.py")
        )
    )

    elevation_mapping_node = Node(
        package="urc_perception",
        executable="urc_perception_ElevationMapping",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("urc_perception"),
                    "config",
                    "mapping_params.yaml",
                ]
            )
        ],
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[
                        load_joint_state_broadcaster,
                        load_drivetrain_controller,
                        teleop_launch,
                    ],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_drivetrain_controller,
                    on_exit=[elevation_mapping_node, twist_mux_node],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=elevation_mapping_node,
                    on_start=[
                        path_planning_launch,
                        trajectory_following_launch,
                        bt_launch,
                    ],
                )
            ),
            enable_color,
            gazebo,
            sim_gps_handler_node,
            load_robot_state_publisher,
            spawn_robot,
            ekf_launch,
        ]
    )
