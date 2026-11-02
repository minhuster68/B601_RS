"""Display the B601-RS model and publish adjustable joint states in RViz."""

from pathlib import Path
import os
import runpy
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "rebot_arm_rs_description"


def _robot_description(package_share):
    """Resolve portable mesh paths and adapt multi-material links for RViz."""
    urdf_path = package_share / "urdf" / "ReBot_Arm_RS.urdf"
    root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        if "://" in filename:
            continue
        # Normalize '..' without following colcon --symlink-install links out
        # of the package share directory and into the source checkout.
        mesh_path = Path(os.path.abspath(urdf_path.parent / filename))
        relative_path = mesh_path.relative_to(package_share.absolute())
        if not mesh_path.is_file():
            raise FileNotFoundError(f"Missing robot mesh: {mesh_path}")
        mesh.set("filename", f"package://{PACKAGE_NAME}/{relative_path.as_posix()}")

    compat = runpy.run_path(str(package_share / "tools" / "rviz_urdf_compat.py"))
    return compat["make_rviz_compatible"](ET.tostring(root, encoding="unicode"))


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    description = ParameterValue(_robot_description(package_share), value_type=str)
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    parameters = {"robot_description": description, "use_sim_time": use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Use joint sliders; false publishes default joint positions.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Start the RViz window.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use /clock only when an external simulator publishes it.",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value=str(package_share / "rviz" / "rs.rviz"),
            description="RViz configuration file.",
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen", parameters=[parameters],
        ),
        Node(
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui", output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
            parameters=[parameters, {"rate": 30}],
        ),
        Node(
            package="joint_state_publisher", executable="joint_state_publisher",
            name="joint_state_publisher", output="screen",
            condition=UnlessCondition(LaunchConfiguration("gui")),
            parameters=[parameters, {"rate": 30}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
