"""MoveIt planning and trajectory execution using mock hardware only."""

from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _robot_description(description_share, moveit_share):
    # Reuse the same mesh paths, materials and mimic joint as display.launch.py.
    loader = runpy.run_path(str(description_share / "launch" / "display.launch.py"))
    root = ET.fromstring(loader["_robot_description"](description_share))
    ET.SubElement(root, "link", {"name": "world"})
    fixed = ET.SubElement(root, "joint", {"name": "world_to_base", "type": "fixed"})
    ET.SubElement(fixed, "parent", {"link": "world"})
    ET.SubElement(fixed, "child", {"link": "base_link"})
    control = xacro.process_file(
        str(moveit_share / "config" / "rebot_arm_rs.ros2_control.xacro"),
        mappings={"initial_positions_file": str(moveit_share / "config" / "initial_positions.yaml")},
    )
    root.append(ET.fromstring(control.toxml()).find("ros2_control"))
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    share = Path(get_package_share_directory("rebot_arm_moveit"))
    description_share = Path(get_package_share_directory("rebot_arm_rs_description"))
    config = share / "config"
    description = {"robot_description": _robot_description(description_share, share)}
    semantic = {"robot_description_semantic": (config / "rebot_arm_rs.srdf").read_text()}
    kinematics = {"robot_description_kinematics": _load_yaml(config / "kinematics.yaml")}
    limits = {"robot_description_planning": _load_yaml(config / "joint_limits.yaml")}
    pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": _load_yaml(config / "ompl_planning.yaml"),
    }
    shared_parameters = [description, semantic, kinematics, limits, pipelines, {"use_sim_time": False}]

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Open MoveIt RViz."),
        DeclareLaunchArgument(
            "rviz_config", default_value=str(config / "moveit.rviz"),
            description="RViz configuration file.",
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen", parameters=[description],
        ),
        Node(
            package="controller_manager", executable="ros2_control_node",
            output="screen", parameters=[str(config / "ros2_controllers.yaml")],
            remappings=[("~/robot_description", "/robot_description")],
        ),
        Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller",
                       "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
        ),
        Node(
            package="moveit_ros_move_group", executable="move_group", output="screen",
            parameters=shared_parameters + [
                _load_yaml(config / "moveit_controllers.yaml"),
                {"allow_trajectory_execution": True,
                 "publish_robot_description_semantic": True,
                 "publish_planning_scene": True,
                 "publish_geometry_updates": True,
                 "publish_state_updates": True,
                 "publish_transforms_updates": True},
            ],
        ),
        Node(
            package="rviz2", executable="rviz2", name="moveit_rviz", output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=shared_parameters,
        ),
    ])
