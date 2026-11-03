"""Manual end-to-end test in a private user/network namespace; never use physical CAN."""

import collections
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import tempfile
import time

import rclpy
from sensor_msgs.msg import JointState
import yaml

root = Path(__file__).resolve().parents[2]
out = Path(tempfile.mkdtemp(prefix='rebot-vcan-'))
print('Validation logs:', out, flush=True)
assert subprocess.run(['ip', 'link', 'show', 'can0'], capture_output=True).returncode != 0, 'Host can0 must not be visible'
subprocess.run(['ip', 'link', 'add', 'vcan_test', 'type', 'vcan'], check=True)
subprocess.run(['ip', 'link', 'set', 'vcan_test', 'up'], check=True)
subprocess.run(['ip', 'link', 'set', 'lo', 'up'], check=True)
configuration = yaml.safe_load((root/'Rebot_Arm_hardware/config/motors.yaml').read_text())
configuration['can_interface'] = 'vcan_test'
configuration['calibration_confirmed'] = True
initial = yaml.safe_load((root/'Rebot_Arm_moveit/config/initial_positions.yaml').read_text())['initial_positions']
motor_position = {}
registers = collections.defaultdict(dict)
running = {}
for i, (name, config) in enumerate(configuration['joints'].items(), 1):
    config.update(motor_id=i, direction=1, motor_zero_rad=0.0,
                  motor_radians_per_joint_unit=100.0 if i == 7 else 1.0, current_limit_a=1.0)
    motor_position[i] = initial[name] * config['motor_radians_per_joint_unit']
    running[i] = False
config_path = out/'vcan_motors.yaml'
config_path.write_text(yaml.safe_dump(configuration))
bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
bus.bind(('vcan_test',))
bus.settimeout(0.01)
stop = threading.Event()
types = collections.Counter()
errors = []

def frame(identifier, payload):
    bus.send(struct.pack('=IB3x8s', 0x80000000 | identifier, 8, payload))

def emulator():
    try:
        while not stop.is_set():
            try:
                message = bus.recv(16)
            except socket.timeout:
                continue
            identifier, dlc, data = struct.unpack('=IB3x8s', message)
            kind = (identifier >> 24) & 31
            motor = identifier & 255
            host = (identifier >> 8) & 255
            if motor not in motor_position or not identifier & 0x80000000:
                continue
            types[kind] += 1
            index = struct.unpack('<H', data[:2])[0]
            if kind == 17:
                if index == 0x7019:
                    value = struct.pack('<f', motor_position[motor])
                elif index == 0x701B:
                    value = struct.pack('<f', 0.0)
                else:
                    value = registers[motor].get(index, bytes(4))
                frame((17 << 24) | (motor << 8) | host, data[:4] + value)
            elif kind == 18:
                registers[motor][index] = data[4:]
                if index == 0x7016:
                    motor_position[motor] = struct.unpack('<f', data[4:])[0]
                frame((2 << 24) | ((2 if running[motor] else 0) << 22) | (motor << 8) | host, bytes(8))
            elif kind in (3, 4):
                running[motor] = kind == 3
                frame((2 << 24) | ((2 if running[motor] else 0) << 22) | (motor << 8) | host, bytes(8))
            else:
                errors.append('Unexpected request type ' + str(kind))
    except Exception as error:
        errors.append(repr(error))

thread = threading.Thread(target=emulator)
thread.start()
process = None
log = None
rclpy.init()
node = rclpy.create_node('vcan_driver_validation')
positions = {}
subscription = node.create_subscription(JointState, '/joint_states', lambda msg: positions.update(zip(msg.name, msg.position)), 10)

def shutdown_launch():
    global process, log
    if process is not None:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        process = None
    if log:
        log.close()
        log = None

def launch(mode, moveit):
    global process, log
    log = (out/f'vcan_{mode}.log').open('w')
    command = ['ros2', 'launch', 'rebot_arm_rs_hardware', 'hardware.launch.py', f'motors_file:={config_path}',
               f'mode:={mode}', f'moveit:={str(moveit).lower()}', 'rviz:=false']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    positions.clear()
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        assert process.poll() is None, 'Launch exited early'
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(positions) >= 8:
            return
    raise AssertionError('Did not receive real-plugin joint feedback')

try:
    probe = subprocess.run(['ros2', 'run', 'rebot_arm_rs_hardware', 'can_probe', '--interface', 'vcan_test', '--scan'],
                           capture_output=True, text=True, timeout=10)
    assert probe.returncode == 0, probe.stdout + probe.stderr
    assert 'Responding motor IDs: 1 2 3 4 5 6 7' in probe.stdout, probe.stdout
    assert set(types) == {17}, types
    print('PASS: can_probe scans IDs 1..254 through a real Linux vcan socket using parameter reads only', flush=True)
    types.clear()
    launch('monitor', False)
    for name, expected in initial.items():
        assert abs(positions[name] - expected) < 1e-5, positions
    shutdown_launch()
    assert set(types) == {17}, types
    assert not any(running.values())
    print('PASS: real RSSystem plugin -> SocketCAN -> 7 emulated motors -> 8 joint states; monitoring sends no enable/stop/write', flush=True)
    launch('control', True)
    smoke = subprocess.Popen(['ros2', 'run', 'rebot_arm_moveit', 'smoke_test.py'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        for line in smoke.stdout:
            print(line.rstrip(), flush=True)
        assert smoke.wait(timeout=5) == 0, 'MoveIt smoke test failed'
    finally:
        if smoke.poll() is None:
            smoke.terminate()
    assert all(registers[m][0x7028] == struct.pack('<I', 4000) for m in range(1,8))
    assert all(registers[m][0x7005] == struct.pack('<I', 5) for m in range(1,8))
    shutdown_launch()
    assert not any(running.values()), running
    assert not errors, errors
    print('PASS: MoveIt -> controllers -> real RSSystem -> SocketCAN -> motor emulator, including CSP/watchdog readback, mimic and shutdown stops', flush=True)
    print('CAN requests observed:', dict(types), flush=True)
finally:
    shutdown_launch()
    node.destroy_node()
    rclpy.try_shutdown()
    stop.set()
    thread.join(timeout=2)
    bus.close()
