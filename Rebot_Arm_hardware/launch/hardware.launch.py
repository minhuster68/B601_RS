"""B601-RS real encoder monitoring or CSP execution through SocketCAN."""

import math
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"Fill a finite number for {label} in motors.yaml")
    return value


def _id(value, label):
    _number(value, label)
    if int(value) != value or not 1 <= value <= 254:
        raise ValueError(f"{label} must be an integer in [1,254]")
    return int(value)


def _build_description(description_share, moveit_share, config, read_only):
    loader = runpy.run_path(str(moveit_share / "launch" / "demo.launch.py"))
    root = ET.fromstring(loader["_robot_description"](description_share, moveit_share))
    for control in root.findall("ros2_control"):
        root.remove(control)
    independent = {joint.get("name"): joint for joint in root.findall("joint")
                   if joint.get("type") in ("revolute", "prismatic") and joint.find("mimic") is None}
    if set(config.get("joints", {})) != set(independent):
        raise ValueError("motors.yaml must map exactly joint1..joint6 and gripper_joint1")
    if type(config.get("calibration_confirmed")) is not bool:
        raise ValueError("calibration_confirmed must be a YAML boolean")
    if not read_only and not config["calibration_confirmed"]:
        raise ValueError("mode:=control requires calibration_confirmed: true in motors.yaml")
    interface = config.get("can_interface")
    if not isinstance(interface, str) or not interface or len(interface.encode()) >= 16:
        raise ValueError("Invalid can_interface in motors.yaml")
    host = _id(config.get("host_id"), "host_id")
    control = ET.SubElement(root, "ros2_control", {"name": "RebotArmRSSystem", "type": "system"})
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "rebot_arm_rs_hardware/RSSystem"

    def parameter(parent, name, value):
        if isinstance(value, bool):
            value = str(value).lower()
        ET.SubElement(parent, "param", {"name": name}).text = str(value)

    parameter(hardware, "can_interface", interface)
    parameter(hardware, "host_id", host)
    parameter(hardware, "read_only", read_only)
    parameter(hardware, "calibration_confirmed", config["calibration_confirmed"])
    for key in ("feedback_timeout_s", "startup_timeout_s", "poll_rate_hz", "max_control_period_s", "watchdog_ms"):
        parameter(hardware, key, _number(config.get(key), key))
    tolerance = _number(config.get("position_limit_tolerance_rad", 0.05), "position_limit_tolerance_rad")
    if tolerance < 0:
        raise ValueError("position_limit_tolerance_rad must be nonnegative")
    parameter(hardware, "position_limit_tolerance_rad", tolerance)

    ids = set()
    for name, source in independent.items():
        motor = config["joints"][name]
        motor_id = _id(motor.get("motor_id"), f"{name}.motor_id")
        if motor_id == host or motor_id in ids:
            raise ValueError(f"Duplicate motor ID or host/motor ID collision at {name}")
        ids.add(motor_id)
        if motor.get("direction") not in (-1, 1) or isinstance(motor.get("direction"), bool):
            raise ValueError(f"Set {name}.direction to +1 or -1")
        joint = ET.SubElement(control, "joint", {"name": name})
        parameter(joint, "motor_id", motor_id)
        for key in ("direction", "motor_zero_rad", "joint_zero", "motor_radians_per_joint_unit",
                    "max_velocity", "max_tracking_error"):
            parameter(joint, key, _number(motor.get(key), f"{name}.{key}"))
        if motor["motor_radians_per_joint_unit"] <= 0 or motor["max_tracking_error"] <= 0:
            raise ValueError(f"{name}: scale and tracking error must be positive")
        original_limit = source.find("limit")
        lower = _number(motor.get("position_min", float(original_limit.get("lower"))), f"{name}.position_min")
        upper = _number(motor.get("position_max", float(original_limit.get("upper"))), f"{name}.position_max")
        if not float(original_limit.get("lower")) <= lower < upper <= float(original_limit.get("upper")):
            raise ValueError(f"{name}: hardware position limits must stay inside the URDF limits")
        if not 0 < motor["max_velocity"] <= float(original_limit.get("velocity")):
            raise ValueError(f"{name}: invalid max_velocity")
        parameter(joint, "position_min", lower)
        parameter(joint, "position_max", upper)
        current = motor.get("current_limit_a")
        if current is None and read_only:
            current = 0.0
        _number(current, f"{name}.current_limit_a")
        if current < 0 or (not read_only and current <= 0):
            raise ValueError(f"{name}: control requires a positive current_limit_a")
        parameter(joint, "current_limit_a", current)
        if not read_only:
            command = ET.SubElement(joint, "command_interface", {"name": "position"})
            parameter(command, "min", lower)
            parameter(command, "max", upper)
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    for source in root.findall("joint"):
        mimic = source.find("mimic")
        if mimic is None:
            continue
        if mimic.get("joint") not in independent:
            raise ValueError("Mimic parent must be an independently controlled motor joint")
        joint = ET.SubElement(control, "joint", {"name": source.get("name")})
        parameter(joint, "mimic", mimic.get("joint"))
        parameter(joint, "multiplier", float(mimic.get("multiplier", "1")))
        parameter(joint, "offset", float(mimic.get("offset", "0")))
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})
    return ET.tostring(root, encoding="unicode")


def _setup(context):
    description_share = Path(get_package_share_directory("rebot_arm_rs_description"))
    moveit_share = Path(get_package_share_directory("rebot_arm_moveit"))
    config_path = Path(LaunchConfiguration("motors_file").perform(context)).expanduser()
    config = yaml.safe_load(config_path.read_text())
    read_only = LaunchConfiguration("mode").perform(context) == "monitor"
    use_moveit = LaunchConfiguration("moveit").perform(context) == "true"
    use_rviz = LaunchConfiguration("rviz").perform(context) == "true"
    # Validation completes before any ROS nodes or CAN sockets are started.
    description = {"robot_description": _build_description(description_share, moveit_share, config, read_only)}
    controllers = ["joint_state_broadcaster"]
    if not read_only:
        controllers.extend(["arm_controller", "gripper_controller"])
    actions = [
        LogInfo(msg=f"SocketCAN {config['can_interface']}: {'READ-ONLY monitoring' if read_only else 'CSP CONTROL'}"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[str(moveit_share / "config" / "ros2_controllers.yaml")],
             remappings=[("~/robot_description", "/robot_description")]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=controllers + ["--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"]),
    ]
    parameters = [description, {"use_sim_time": False}]
    if use_moveit:
        def load(name):
            return yaml.safe_load((moveit_share / "config" / name).read_text())
        planning_limits = load("joint_limits.yaml")
        for name, motor in config["joints"].items():
            planning_limits["joint_limits"][name]["max_velocity"] = min(
                planning_limits["joint_limits"][name]["max_velocity"], motor["max_velocity"])
        planning_limits["joint_limits"]["gripper_joint2"]["max_velocity"] = (
            planning_limits["joint_limits"]["gripper_joint1"]["max_velocity"])
        parameters.extend([
            {"robot_description_semantic": (moveit_share / "config" / "rebot_arm_rs.srdf").read_text()},
            {"robot_description_kinematics": load("kinematics.yaml")},
            {"robot_description_planning": planning_limits},
            {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl", "ompl": load("ompl_planning.yaml")},
        ])
        execution = load("moveit_controllers.yaml") if not read_only else {}
        actions.append(Node(
            package="moveit_ros_move_group", executable="move_group", output="screen",
            parameters=parameters + [execution, {
                "allow_trajectory_execution": not read_only,
                "publish_robot_description_semantic": True, "publish_planning_scene": True,
                "publish_geometry_updates": True, "publish_state_updates": True,
                "publish_transforms_updates": True,
            }],
        ))
    if use_rviz:
        rviz_file = moveit_share / "config" / "moveit.rviz" if use_moveit else description_share / "rviz" / "rs.rviz"
        actions.append(Node(package="rviz2", executable="rviz2", name="hardware_rviz",
                            arguments=["-d", str(rviz_file)], parameters=parameters, output="screen"))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("rebot_arm_rs_hardware"))
    return LaunchDescription([
        DeclareLaunchArgument("motors_file", default_value=str(share / "config" / "motors.yaml"),
                              description="YAML with actual motor IDs, calibration and limits."),
        DeclareLaunchArgument("mode", default_value="monitor", choices=["monitor", "control"],
                              description="Read-only encoder monitoring or calibrated CSP motor control."),
        DeclareLaunchArgument("moveit", default_value="true", choices=["true", "false"],
                              description="Run MoveIt; trajectory execution is disabled in monitor mode."),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"],
                              description="Open RViz visualization."),
        OpaqueFunction(function=_setup),
    ])
