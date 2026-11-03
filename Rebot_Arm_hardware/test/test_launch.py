"""Hardware launch configuration tests. Never starts ROS nodes or opens CAN."""

from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
MOVEIT = PACKAGE.parent / "Rebot_Arm_moveit"
DESCRIPTION = PACKAGE.parent / "Rebot_Arm_description" / "RS_description"
LOADER = runpy.run_path(str(PACKAGE / "launch" / "hardware.launch.py"))


@pytest.fixture
def description(tmp_path):
    for directory in ("launch", "meshes", "urdf"):
        (tmp_path / directory).symlink_to(DESCRIPTION / directory, target_is_directory=True)
    (tmp_path / "tools").symlink_to(DESCRIPTION.parent / "tools", target_is_directory=True)
    return tmp_path


def config():
    result = yaml.safe_load((PACKAGE / "config" / "motors.yaml").read_text())
    result["calibration_confirmed"] = False
    for index, motor in enumerate(result["joints"].values(), start=1):
        motor.update(motor_id=index, direction=1, motor_zero_rad=0.0,
                     motor_radians_per_joint_unit=100.0 if index == 7 else 1.0,
                     current_limit_a=1.0)
    return result


def build(description, configuration, monitor):
    return ET.fromstring(LOADER["_build_description"](description, MOVEIT, configuration, monitor))


def test_config_allows_monitoring_but_blocks_control_until_confirmed(description):
    template = yaml.safe_load((PACKAGE / "config" / "motors.yaml").read_text())
    # Exercise both states regardless of the owner's current commissioning flag.
    template["calibration_confirmed"] = False
    robot = build(description, template, True)
    assert not robot.findall("ros2_control/joint/command_interface")
    with pytest.raises(ValueError, match="calibration_confirmed"):
        build(description, template, False)
    template["calibration_confirmed"] = True
    assert len(build(description, template, False).findall("ros2_control/joint/command_interface")) == 7
    template["joints"]["joint1"]["current_limit_a"] = None
    with pytest.raises(ValueError, match="joint1.current_limit_a"):
        build(description, template, False)


def test_monitor_has_only_state_interfaces_and_preserves_mimic(description):
    robot = build(description, config(), True)
    control = robot.find("ros2_control")
    assert len(robot.findall("ros2_control")) == 1
    assert control.find("hardware/plugin").text == "rebot_arm_rs_hardware/RSSystem"
    assert control.find("hardware/param[@name='read_only']").text == "true"
    assert control.find("hardware/param[@name='position_limit_tolerance_rad']").text == "0.05"
    assert not control.findall(".//command_interface")
    assert not control.findall(".//param[@name='initial_value']")
    assert len(control.findall("joint")) == 8
    follower = control.find("joint[@name='gripper_joint2']")
    assert follower.find("param[@name='motor_id']") is None
    assert follower.find("param[@name='mimic']").text == "gripper_joint1"
    assert robot.find("joint[@name='gripper_joint2']/mimic").get("multiplier") == "1"


def test_control_needs_calibration_and_has_seven_motor_commands(description):
    configuration = config()
    with pytest.raises(ValueError, match="calibration_confirmed"):
        build(description, configuration, False)
    configuration["calibration_confirmed"] = True
    control = build(description, configuration, False).find("ros2_control")
    assert len(control.findall(".//command_interface")) == 7
    assert control.find("hardware/param[@name='read_only']").text == "false"


@pytest.mark.parametrize("change", ["duplicate_id", "host_collision", "bad_direction", "missing_scale", "wide_limits", "zero_current"])
def test_invalid_motor_configuration_is_rejected(description, change):
    configuration = config()
    configuration["calibration_confirmed"] = True
    motor = configuration["joints"]["joint2"]
    if change == "duplicate_id":
        motor["motor_id"] = 1
    elif change == "host_collision":
        motor["motor_id"] = 253
    elif change == "bad_direction":
        motor["direction"] = 0
    elif change == "missing_scale":
        motor["motor_radians_per_joint_unit"] = None
    elif change == "wide_limits":
        motor["position_min"] = -999
    elif change == "zero_current":
        motor["current_limit_a"] = 0
    with pytest.raises(ValueError):
        build(description, configuration, False)


@pytest.mark.parametrize("value", [-0.05, float("nan"), float("inf"), True, None])
def test_invalid_position_limit_tolerance_is_rejected_before_launch(description, value):
    configuration = config()
    configuration["position_limit_tolerance_rad"] = value
    with pytest.raises(ValueError, match="position_limit_tolerance_rad"):
        build(description, configuration, True)
