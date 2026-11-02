"""Cross-check planning semantics and controllers against the real RS URDF."""

from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / "Rebot_Arm_description" / "RS_description"


def load_yaml(name):
    return yaml.safe_load((PACKAGE / "config" / name).read_text())


def test_controllers_cover_exactly_the_independent_joints():
    robot = ET.parse(DESCRIPTION / "urdf" / "ReBot_Arm_RS.urdf").getroot()
    independent = {j.get("name") for j in robot.findall("joint")
                   if j.get("type") != "fixed" and j.find("mimic") is None}
    moveit = load_yaml("moveit_controllers.yaml")["moveit_simple_controller_manager"]
    control = load_yaml("ros2_controllers.yaml")
    controlled = []
    for name in moveit["controller_names"]:
        joints = moveit[name]["joints"]
        assert joints == control[name]["ros__parameters"]["joints"]
        controlled.extend(joints)
    assert len(controlled) == len(set(controlled)) == 7
    assert set(controlled) == independent


def test_named_and_initial_positions_obey_urdf_limits():
    robot = ET.parse(DESCRIPTION / "urdf" / "ReBot_Arm_RS.urdf").getroot()
    limits = {j.get("name"): j.find("limit") for j in robot.findall("joint") if j.find("limit") is not None}
    srdf = ET.parse(PACKAGE / "config" / "rebot_arm_rs.srdf").getroot()
    assert srdf.get("name") == robot.get("name")
    samples = list(load_yaml("initial_positions.yaml")["initial_positions"].items())
    for state in srdf.findall("group_state"):
        samples.extend((j.get("name"), float(j.get("value"))) for j in state.findall("joint"))
    for name, value in samples:
        assert float(limits[name].get("lower")) <= value <= float(limits[name].get("upper"))
    for name, limit in load_yaml("joint_limits.yaml")["joint_limits"].items():
        assert 0 < limit["max_velocity"] <= float(limits[name].get("velocity"))
        assert limit["max_acceleration"] > 0


def test_mock_control_and_moveit_share_mimic_and_robot_model(tmp_path):
    for directory in ("launch", "meshes", "urdf"):
        (tmp_path / directory).symlink_to(DESCRIPTION / directory, target_is_directory=True)
    (tmp_path / "tools").symlink_to(DESCRIPTION.parent / "tools", target_is_directory=True)
    loader = runpy.run_path(str(PACKAGE / "launch" / "demo.launch.py"))
    robot = ET.fromstring(loader["_robot_description"](tmp_path, PACKAGE))
    hardware = robot.find("ros2_control")
    assert hardware.find("hardware/plugin").text == "mock_components/GenericSystem"
    follower = hardware.find("joint[@name='gripper_joint2']")
    assert follower.find("command_interface") is None
    assert follower.find("param[@name='mimic']").text == "gripper_joint1"
    assert robot.find("joint[@name='gripper_joint2']/mimic").get("multiplier") == "1"
    assert robot.find("joint[@name='world_to_base']/parent").get("link") == "world"
    assert all(len(link.findall("visual")) <= 1 for link in robot.findall("link"))
    for mesh in robot.findall(".//mesh"):
        assert mesh.get("filename").startswith("package://rebot_arm_rs_description/")
