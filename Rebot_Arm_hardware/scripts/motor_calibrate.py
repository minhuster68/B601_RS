#!/usr/bin/env python3
"""Explicit one-time RobStride calibration; zero never enables a motor."""

import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import socket
import struct
import time
import tempfile
from zoneinfo import ZoneInfo


EFF = 0x80000000
RTR = 0x40000000
ERR = 0x20000000
POSITION, VELOCITY, IQ, CURRENT_LIMIT = 0x7019, 0x701B, 0x701A, 0x7018
MODE, ZERO_FLAG, MECHANICAL_OFFSET = 0x7005, 0x7029, 0x2005


class Bus:
    def __init__(self, interface, host):
        self.host = host
        self.last_tx = 0.0
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                   struct.pack("=II", EFF | host, EFF | RTR | 0xFF))
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER,
                                   struct.pack("=I", 0x40 | 0x04 | 0x100))
            self.socket.settimeout(0.05)
            self.socket.bind((interface,))
        except BaseException:
            self.socket.close()
            raise

    def close(self):
        self.socket.close()

    def exchange(self, kind, motor, payload, reply_kind, index=None):
        if not 1 <= motor <= 254 or motor == self.host or len(payload) != 8:
            raise ValueError("Invalid CAN request")
        identifier = EFF | (kind << 24) | (self.host << 8) | motor
        time.sleep(max(0.0, 0.01 - (time.monotonic() - self.last_tx)))
        self.socket.send(struct.pack("=IB3x8s", identifier, 8, payload))
        self.last_tx = time.monotonic()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            try:
                raw = self.socket.recv(16)
            except socket.timeout:
                continue
            if len(raw) != 16:
                raise RuntimeError("Malformed CAN frame")
            identifier, length, data = struct.unpack("=IB3x8s", raw)
            if identifier & ERR:
                raise RuntimeError("CAN bus/controller error")
            if not identifier & EFF or identifier & RTR or length != 8:
                continue
            if (identifier & 0xFF) != self.host or ((identifier >> 8) & 0xFF) != motor:
                continue
            response_type = (identifier >> 24) & 0x1F
            flags = (identifier >> 16) & 0xFF
            if response_type in (2, 24) and flags & 0x3F:
                raise RuntimeError(f"Motor {motor} fault bits: {flags & 0x3F}")
            if response_type == 21 and data != bytes(8):
                raise RuntimeError(f"Motor {motor} fault report: {data.hex()}")
            if response_type != reply_kind:
                continue
            if reply_kind == 17:
                if struct.unpack("<H", data[:2])[0] != index:
                    continue
                if flags or data[2:4] != bytes(2):
                    raise RuntimeError(f"Motor {motor} rejected read 0x{index:04x}")
            elif flags >> 6 != 0:
                raise RuntimeError(f"Motor {motor} is not stopped; aborting zero procedure")
            return data
        raise RuntimeError(f"Motor {motor}: response timeout for type {kind}, index {index}")

    def read(self, motor, index, fmt="f"):
        for attempt in range(3):
            try:
                data = self.exchange(17, motor, struct.pack("<HHI", index, 0, 0), 17, index)
                break
            except RuntimeError as error:
                if "response timeout" not in str(error) or attempt == 2:
                    raise
        value = struct.unpack("<" + fmt, data[4:4 + struct.calcsize("<" + fmt)])[0]
        if not math.isfinite(value):
            raise RuntimeError("Non-finite motor parameter")
        return value

    def write_u8(self, motor, index, value):
        if self.read(motor, index, "B") == value:
            return
        self.exchange(18, motor, struct.pack("<HHBBBB", index, 0, value, 0, 0, 0), 2)
        if self.read(motor, index, "B") != value:
            raise RuntimeError(f"Motor {motor}: write/readback mismatch at 0x{index:04x}")


def snapshot(bus, ids):
    result = {}
    for motor in ids:
        result[str(motor)] = {
        "position_rad": bus.read(motor, POSITION),
        "velocity_rad_s": bus.read(motor, VELOCITY),
        "iq_filtered_a": bus.read(motor, IQ),
        "current_limit_a": bus.read(motor, CURRENT_LIMIT),
        "run_mode": bus.read(motor, MODE, "B"),
        "zero_flag": bus.read(motor, ZERO_FLAG, "B"),
        }
        # Some firmware exposes only the 0x70xx parameter bank through type 17.
        # Preserve the old coordinate using mechPos even when 0x2005 is unavailable;
        # never write this magnetic-encoder offset register.
        try:
            result[str(motor)]["mechanical_offset_rad"] = bus.read(motor, MECHANICAL_OFFSET)
        except RuntimeError as error:
            if "rejected read 0x2005" not in str(error):
                raise
            result[str(motor)]["mechanical_offset_rad"] = None
    return result


def save_report(path, report, initial=False):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    if initial:
        stream = path.open("x", encoding="utf-8")
        temporary = None
    else:
        stream = tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False)
        temporary = Path(stream.name)
    with stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    if temporary:
        os.replace(temporary, path)


def zero_motors(bus, ids, backup, apply=False, allow_unacknowledged_save=False,
                startup_range="signed"):
    if startup_range not in ("signed", "unsigned"):
        raise ValueError("startup_range must be signed or unsigned")
    if 7 in ids and startup_range != "unsigned":
        raise ValueError("B601 gripper ID 7 requires --startup-range unsigned for its 5.96301 rad travel")
    if startup_range == "unsigned" and any(motor != 7 for motor in ids):
        raise ValueError("Unsigned startup is reserved for gripper ID 7; zero arm joints separately")
    zero_flag = 1 if startup_range == "signed" else 0
    before = snapshot(bus, ids)
    report = {"at_local": datetime.now(ZoneInfo("Asia/Ho_Chi_Minh")).isoformat(),
              "motor_ids": ids, "before": before, "completed": [],
              "startup_range": startup_range, "startup_zero_flag": zero_flag,
              "zeroed_ids": [], "save_sent_ids": [], "save_acknowledged": {},
              "power_cycle_verified": False}
    if not apply:
        print(json.dumps(report, indent=2, allow_nan=False))
        print("Preview only. No stop/zero/save commands sent.")
        return report
    if not backup:
        raise ValueError("--backup is required with --apply")
    # A durable before-snapshot must exist before the first modifying frame.
    save_report(backup, report, initial=True)
    try:
        # Actual RS feedback exhibited velocity spikes while encoder position
        # varied by <0.0003 rad. Check a position window instead of one noisy
        # derivative sample. Any drift/oscillation >0.002 rad aborts all writes.
        positions = {motor: [before[str(motor)]["position_rad"]] for motor in ids}
        report["stationarity_span_rad"] = {}
        for _ in range(5):
            time.sleep(0.05)
            for motor in ids:
                positions[motor].append(bus.read(motor, POSITION))
                span = max(positions[motor]) - min(positions[motor])
                report["stationarity_span_rad"][str(motor)] = span
                if span > 0.002:
                    raise RuntimeError(f"Motor {motor} position drift {span:.6f} rad; zero aborted before any writes")
        # All selected joints are supported at their known URDF-zero pose.
        for motor in ids:
            bus.exchange(4, motor, bytes(8), 2)
        for motor in ids:
            bus.write_u8(motor, MODE, 5)
            # Arm startup angles include negatives; gripper travel exceeds pi.
            bus.write_u8(motor, ZERO_FLAG, zero_flag)
        for motor in ids:
            bus.exchange(6, motor, bytes([1, 0, 0, 0, 0, 0, 0, 0]), 2)
            time.sleep(0.05)
            position = bus.read(motor, POSITION)
            if abs(position) > 0.005:
                raise RuntimeError(f"Motor {motor} did not reach encoder zero: {position}")
            report["zeroed_ids"].append(motor)
            report["save_sent_ids"].append(motor)
            save_report(backup, report)
            try:
                bus.exchange(22, motor, bytes(range(1, 9)), 2)
                report["save_acknowledged"][str(motor)] = True
            except RuntimeError as error:
                report["save_acknowledged"][str(motor)] = False
                if not allow_unacknowledged_save or "response timeout" not in str(error):
                    raise
            time.sleep(0.2)
            after = snapshot(bus, [motor])[str(motor)]
            if abs(after["position_rad"]) > 0.005 or after["zero_flag"] != zero_flag:
                raise RuntimeError(f"Motor {motor} failed zero/save readback")
            report["completed"].append(motor)
            report.setdefault("after", {})[str(motor)] = after
            save_report(backup, report)
            saved = "save acknowledged" if report["save_acknowledged"][str(motor)] else "save sent once, no ACK; power-cycle verification required"
            print(f"Motor {motor}: zero readback={after['position_rad']:.6f} rad; {saved}; remains stopped", flush=True)
        return report
    except BaseException as error:
        report["error"] = str(error)
        save_report(backup, report)
        raise


def gripper_scale(theta1, theta2, gap1_mm, gap2_mm):
    values = (theta1, theta2, gap1_mm, gap2_mm)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("All measurements must be finite")
    if gap1_mm < 0 or gap2_mm < 0 or abs(gap2_mm - gap1_mm) < 0.1 or abs(theta2 - theta1) < 1e-6:
        raise ValueError("Use two different measured motor angles and jaw gaps")
    # The gap changes twice as far as a single finger's URDF coordinate.
    signed = 2 * (theta2 - theta1) / ((gap2_mm - gap1_mm) / 1000)
    return {"motor_radians_per_joint_unit": abs(signed), "direction": 1 if signed > 0 else -1}


def parse_ids(text, host):
    ids = [int(item) for item in text.split(",")]
    if not ids or len(ids) != len(set(ids)) or any(not 1 <= motor <= 254 or motor == host for motor in ids):
        raise ValueError("Provide unique motor IDs in [1,254], different from host")
    return ids


def current_log(bus, motor, seconds, rate, output):
    if not math.isfinite(seconds) or not 0 < seconds <= 3600 or not math.isfinite(rate) or not 1 <= rate <= 20:
        raise ValueError("Use seconds in (0,3600] and rate in [1,20] Hz")
    values = []
    start = time.monotonic()
    with Path(output).expanduser().open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "motor_id", "position_rad", "velocity_rad_s", "iq_filtered_a", "configured_current_limit_a"])
        while time.monotonic() - start < seconds:
            sample_at = time.monotonic()
            position = bus.read(motor, POSITION)
            velocity = bus.read(motor, VELOCITY)
            current = bus.read(motor, IQ)
            limit = bus.read(motor, CURRENT_LIMIT)
            writer.writerow([time.monotonic() - start, motor, position, velocity, current, limit])
            stream.flush()
            values.append(abs(current))
            time.sleep(max(0, 1 / rate - (time.monotonic() - sample_at)))
    print(json.dumps({"samples": len(values), "max_observed_abs_iq_a": max(values) if values else None,
                      "note": "Observed filtered current only; this is not a recommended current limit."}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("inspect", "zero", "current-log"):
        command = sub.add_parser(name)
        command.add_argument("--interface", default="can0")
        command.add_argument("--host", type=int, default=253)
        command.add_argument("--ids", required=True)
        if name == "zero":
            command.add_argument("--startup-range", choices=("signed", "unsigned"), default="signed",
                                 help="Signed for arm joints; unsigned required for gripper ID 7")
            command.add_argument("--apply", action="store_true", help="Stop, set zero and save; requires supported, known URDF-zero pose")
            command.add_argument("--backup", help="New JSON backup path; an existing file is never overwritten initially")
            command.add_argument("--allow-unacknowledged-save", action="store_true",
                                 help="Only after this firmware's retention is verified; never retries save and still requires power-cycle verification")
        if name == "current-log":
            command.add_argument("--seconds", type=float, default=10)
            command.add_argument("--rate", type=float, default=10)
            command.add_argument("--output", required=True, help="New CSV path")
    scale = sub.add_parser("gripper-scale")
    scale.add_argument("--theta1", type=float, required=True)
    scale.add_argument("--theta2", type=float, required=True)
    scale.add_argument("--gap1-mm", type=float, required=True)
    scale.add_argument("--gap2-mm", type=float, required=True)
    args = parser.parse_args()
    if args.command == "gripper-scale":
        print(json.dumps(gripper_scale(args.theta1, args.theta2, args.gap1_mm, args.gap2_mm), indent=2))
        return
    if not 1 <= args.host <= 254:
        parser.error("Host ID must be in [1,254]")
    ids = parse_ids(args.ids, args.host)
    if args.command == "current-log" and len(ids) != 1:
        parser.error("current-log reads one motor at a time")
    if args.command == "zero" and args.apply and not args.backup:
        parser.error("--backup is required with --apply")
    bus = Bus(args.interface, args.host)
    try:
        if args.command == "inspect":
            print(json.dumps(snapshot(bus, ids), indent=2, allow_nan=False))
        elif args.command == "current-log":
            current_log(bus, ids[0], args.seconds, args.rate, args.output)
        else:
            zero_motors(bus, ids, args.backup, args.apply, args.allow_unacknowledged_save,
                        args.startup_range)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
