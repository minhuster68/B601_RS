"""Calibration tests never connect to physical motors."""

import json
from pathlib import Path
import runpy

import pytest

MODULE = runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts" / "motor_calibrate.py"))


class MotorBus:
    def __init__(self):
        self.positions = {motor: 0.5 * motor for motor in range(1, 8)}
        self.writes = []
        self.fail_zero = None
        self.flags = {motor: 0 for motor in self.positions}

    def read(self, motor, index, fmt="f"):
        if index == MODULE["POSITION"]:
            return self.positions[motor]
        if index == MODULE["VELOCITY"]:
            return 0.0
        if index == MODULE["CURRENT_LIMIT"]:
            return 1.0
        if index == MODULE["IQ"]:
            return 0.0
        if index == MODULE["MODE"]:
            return 5
        if index == MODULE["ZERO_FLAG"]:
            return self.flags[motor]
        if index == MODULE["MECHANICAL_OFFSET"]:
            raise RuntimeError(f"Motor {motor} rejected read 0x2005")
        raise AssertionError(index)

    def write_u8(self, motor, index, value):
        self.writes.append((18, motor, index, value))
        assert index in (MODULE["MODE"], MODULE["ZERO_FLAG"])
        if index == MODULE["ZERO_FLAG"]:
            self.flags[motor] = value

    def exchange(self, kind, motor, payload, reply_kind, index=None):
        self.writes.append((kind, motor, payload))
        assert reply_kind == 2
        assert kind in (4, 6, 22)
        if kind == 6:
            assert payload == bytes([1, 0, 0, 0, 0, 0, 0, 0])
            if motor == self.fail_zero:
                raise RuntimeError("Injected zero failure")
            self.positions[motor] = 0.0
        if kind == 22:
            assert payload == bytes(range(1, 9))
        return bytes(8)


@pytest.fixture(autouse=True)
def no_delays(monkeypatch):
    monkeypatch.setattr(MODULE["time"], "sleep", lambda _: None)


def test_preview_only_reads_and_does_not_write():
    bus = MotorBus()
    result = MODULE["zero_motors"](bus, [1, 2, 3, 4, 5, 6], None)
    assert not bus.writes
    assert result["completed"] == []
    assert result["before"]["1"]["mechanical_offset_rad"] is None


def test_zero_has_durable_backup_stays_stopped_and_leaves_gripper_alone(tmp_path):
    bus = MotorBus()
    backup = tmp_path / "zero.json"
    result = MODULE["zero_motors"](bus, [1, 2, 3, 4, 5, 6], backup, True)
    saved = json.loads(backup.read_text())
    assert saved["before"]["1"]["position_rad"] == 0.5
    assert saved["completed"] == [1, 2, 3, 4, 5, 6]
    assert saved["power_cycle_verified"] is False
    assert result["after"]["1"]["position_rad"] == 0.0
    assert bus.positions[7] == 3.5
    assert all(command[1] != 7 for command in bus.writes)
    assert all(command[0] != 3 for command in bus.writes)
    assert [command[0] for command in bus.writes[:6]] == [4] * 6


def test_existing_backup_prevents_all_modifications(tmp_path):
    bus = MotorBus()
    backup = tmp_path / "existing.json"
    backup.write_text("original")
    with pytest.raises(FileExistsError):
        MODULE["zero_motors"](bus, [1], backup, True)
    assert not bus.writes
    assert backup.read_text() == "original"


def test_gripper_zero_uses_unsigned_startup_and_leaves_arm_unchanged(tmp_path):
    bus = MotorBus()
    arms = {motor: bus.positions[motor] for motor in range(1, 7)}
    result = MODULE["zero_motors"](bus, [7], tmp_path / "gripper.json", True,
                                  startup_range="unsigned")
    assert result["startup_zero_flag"] == 0
    assert result["after"]["7"]["zero_flag"] == 0
    assert result["after"]["7"]["position_rad"] == 0
    assert all(command[1] == 7 for command in bus.writes)
    assert all(command[0] != 3 for command in bus.writes)
    assert arms == {motor: bus.positions[motor] for motor in range(1, 7)}


@pytest.mark.parametrize("ids,startup_range", [([7], "signed"), ([1, 7], "unsigned"), ([1], "bad")])
def test_incorrect_startup_range_rejected_before_writes(tmp_path, ids, startup_range):
    bus = MotorBus()
    with pytest.raises(ValueError):
        MODULE["zero_motors"](bus, ids, tmp_path / "bad.json", True, startup_range=startup_range)
    assert not bus.writes


def test_position_drift_aborts_before_stopping_or_zeroing(tmp_path):
    class Drifting(MotorBus):
        def read(self, motor, index, fmt="f"):
            if index == MODULE["POSITION"]:
                self.positions[motor] += 0.001
            return super().read(motor, index, fmt)
    bus = Drifting()
    with pytest.raises(RuntimeError, match="position drift"):
        MODULE["zero_motors"](bus, [1, 2], tmp_path / "moving.json", True)
    assert not bus.writes


def test_partial_failure_is_recorded_without_automatic_rollback(tmp_path):
    bus = MotorBus()
    bus.fail_zero = 2
    backup = tmp_path / "partial.json"
    with pytest.raises(RuntimeError, match="Injected"):
        MODULE["zero_motors"](bus, [1, 2, 3], backup, True)
    result = json.loads(backup.read_text())
    assert result["completed"] == [1]
    assert "error" in result
    assert bus.positions[3] == 1.5
    assert not any(command[0] == 3 for command in bus.writes)


def test_gripper_uses_half_the_change_in_full_jaw_gap():
    result = MODULE["gripper_scale"](1.0, 3.0, 10.0, 30.0)
    assert result == {"motor_radians_per_joint_unit": 200.0, "direction": 1}
    assert MODULE["gripper_scale"](3.0, 1.0, 10.0, 30.0)["direction"] == -1


@pytest.mark.parametrize("allow", [False, True])
def test_save_timeout_never_retries_and_requires_retention_verification(tmp_path, allow):
    class NoSaveAck(MotorBus):
        def exchange(self, kind, motor, payload, reply_kind, index=None):
            result = super().exchange(kind, motor, payload, reply_kind, index)
            if kind == 22:
                raise RuntimeError("response timeout")
            return result
    bus = NoSaveAck()
    backup = tmp_path / "noack.json"
    if allow:
        MODULE["zero_motors"](bus, [1, 2], backup, True, True)
    else:
        with pytest.raises(RuntimeError, match="response timeout"):
            MODULE["zero_motors"](bus, [1, 2], backup, True)
    result = json.loads(backup.read_text())
    assert result["power_cycle_verified"] is False
    assert result["save_acknowledged"]["1"] is False
    assert sum(command[0] == 22 and command[1] == 1 for command in bus.writes) == 1
    assert (2 in result["zeroed_ids"]) is allow


@pytest.mark.parametrize("values", [(1, 1, 0, 10), (1, 2, 10, 10), (1, 2, -1, 10)])
def test_bad_gripper_measurements_are_rejected(values):
    with pytest.raises(ValueError):
        MODULE["gripper_scale"](*values)


def test_current_log_only_reads_and_does_not_recommend_a_limit(tmp_path, monkeypatch, capsys):
    times = iter([0.0, 0.01, 0.01, 0.02, 0.02, 1.0])
    monkeypatch.setattr(MODULE["time"], "monotonic", lambda: next(times))
    bus = MotorBus()
    output = tmp_path / "current.csv"
    MODULE["current_log"](bus, 1, 0.5, 10, output)
    assert not bus.writes
    assert "iq_filtered_a" in output.read_text()
    assert len(output.read_text().splitlines()) == 2
    assert "not a recommended current limit" in capsys.readouterr().out
