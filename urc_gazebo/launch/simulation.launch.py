from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_urc_gazebo = get_package_share_directory("urc_gazebo")
    world_path = os.path.join(pkg_urc_gazebo, "urdf/worlds/urc_world.world")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        ),
        launch_arguments={"world": world_path}.items(),
    )

    walli = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_urc_gazebo, "launch", "spawn_walli.py")
        )
    )

    control = Node(
        package="urc_gazebo",
        executable="urc_gazebo_Control",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("urc_gazebo"), "config", "control_params.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            (
                "/control/right_wheel_shock_controller/command",
                "/right_wheel_shock_controller/command",
            ),
            (
                "/control/left_wheel_shock_controller/command",
                "/left_wheel_shock_controller/command",
            ),
            (
                "/control/right_wheel_effort_controller/command",
                "/right_wheel_effort_controller/command",
            ),
            (
                "/control/left_wheel_effort_controller/command",
                "/left_wheel_effort_controller/command",
            ),
            ("/control/robot_enabled", "/robot_enabled"),
            ("/control/encoders", "/encoders"),
            ("/control/motors", "/motors"),
            ("/control/joint_states", "/joint_states"),
        ],
    )

    ground_truth = Node(
        package="urc_gazebo",
        executable="urc_gazebo_GroundTruth",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("urc_gazebo"), "config", "ground_truth_params.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/ground_truth/odometry/filtered", "/odometry/filtered"),
            ("/ground_truth/ground_truth", "/ground_truth"),
            ("/ground_truth/ground_truth/state_raw", "/ground_truth/state_raw"),
        ],
    )

    aruco_detector = Node(
        package="urc_perception",
        executable="urc_perception_ArucoDetector",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("urc_perception"),
                    "config",
                    "aruco_detector_params.yaml",
                ]
            )
        ],
        remappings=[("/aruco_detector/aruco_detection", "/aruco_detection")],
    )

    aruco_location = Node(
        package="urc_perception",
        executable="urc_perception_ArucoLocation",
        output="screen",
        remappings=[("/aruco_location/aruco_location", "/aruco_location")],
    )

    ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world_path,
        ],
        output="screen",
    )

    return LaunchDescription(
        [gazebo, walli, control, ground_truth, aruco_detector, aruco_location]
    )
