import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    normal_lib_names = ["liburc_bt_nodes.so"]
    ros_lib_names = [
        "libbt_call_generate_plan.so",
        "libbt_call_trigger.so",
        "libbt_follow_path.so",
        "libbt_update_current_pose.so",
        "libbt_status_light_publisher.so",
    ]
    node_lib_path_base = os.path.join(
        Path(get_package_share_directory("urc_bt_nodes")).parent.parent.absolute(),
        "lib",
    )

    normal_lib_paths = [
        os.path.join(node_lib_path_base, lib_name) for lib_name in normal_lib_names
    ]
    ros_lib_paths = [
        os.path.join(node_lib_path_base, lib_name) for lib_name in ros_lib_names
    ]

    bt_file_name = "bt_test.xml"
    orchestrator = Node(
        package="urc_behavior",
        executable="urc_behavior_BehaviorTreeOrchestrator",
        parameters=[
            {
                "normal_node_lib_dirs": normal_lib_paths,
                "ros_node_lib_dirs": ros_lib_paths,
                "tree_file_dir": PathJoinSubstitution(
                    [FindPackageShare("urc_behavior"), "strategies", bt_file_name]
                ),
                "tick_rate": 10,
            }
        ],
    )

    enable_color = SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1")

    return LaunchDescription(
        [
            enable_color,
            orchestrator,
        ]
    )
