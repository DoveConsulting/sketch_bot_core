import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # Command-line arguments
    db_arg = DeclareLaunchArgument("db",
                                   default_value="False",
                                   description="Database flag")
    ar_model_arg = DeclareLaunchArgument(
        "ar_model",
        default_value="mk3",
        choices=["mk1", "mk2", "mk3"],
        description="Model of AR4",
    )
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix_arg = DeclareLaunchArgument("tf_prefix",
                                          default_value="",
                                          description="Prefix for AR4 tf_tree")
    tf_prefix = LaunchConfiguration("tf_prefix")

    moveit_config = (
        MoveItConfigsBuilder("ar", package_name="annin_ar4_moveit_config")
        .robot_description(file_path="config/ar.urdf.xacro")
        .robot_description_semantic(file_path="config/ar.srdf.xacro")
        .trajectory_execution(file_path="config/controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp"]
        )
        .to_moveit_configs()
    )

    # use_sim_time={"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    # config_dict.update(use_sim_time)

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )


    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("sketch_bot_core"), "rviz")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[config_dict],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
        ],
    )

    return LaunchDescription([
        db_arg,
        ar_model_arg,
        tf_prefix_arg,
        run_move_group_node,
        rviz_node,
    ])
