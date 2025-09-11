import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import SetLaunchConfiguration

def generate_launch_description() -> LaunchDescription:
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

    sketch_node = GroupAction(
        actions=[
            Node(
                package="sketch_bot_core",
                executable="sketch",
                output="both",
                parameters=[
                    moveit_config.to_dict(),
                ],
            ),
        ],
    )

    plot_node = GroupAction(
        actions=[
            Node(
                package="sketch_bot_core",
                executable="plot",
                output="both",
                parameters=[
                    moveit_config.to_dict(),
                ],
            ),
        ],
    )


    nodes_to_start = [sketch_node, plot_node]
    
    return LaunchDescription(nodes_to_start)