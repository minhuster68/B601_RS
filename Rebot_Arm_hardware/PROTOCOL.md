# Wire protocol and provenance

This driver implements the RobStride **private protocol, classic CAN with
29-bit extended identifiers**, using CSP position mode (`run_mode=5`). It does
not implement the separate 11-bit MIT or CANopen protocols and never changes
the motor's protocol, CAN ID, persistent zero, baud rate or flash parameters.

The separate `motor_calibrate.py` utility provides explicit one-time zero/save
operations, outside the runtime hardware plugin. It is read-only by default;
see `CALIBRATION.md`. Runtime activation never invokes it.

Primary references checked during implementation:

- [Vendor protocol constants](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py)
- [Vendor SocketCAN sample](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/bus.py)
- [RS00 manual, 2026-07-13](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/CAN2.0/RS00/RS00User%20Manual260713.pdf)
- [RS06 manual, 2026-07-13](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/CAN2.0/RS06/RS06User%20Manual260713.pdf)

The implementation is written here; vendor source files/manuals are not copied
into this package. Confirm these registers against the firmware on your motors.

## Address and payload

Requests: `(type << 24) | (host_id << 8) | motor_id`, plus Linux `CAN_EFF_FLAG`.
Responses: host ID in bits 0–7; motor ID in bits 8–15.
Only eight-byte extended data frames for our host/configured motor IDs are
accepted. Parameter replies carry the echoed index; read failures in bits
16–23 are rejected. Frames for other nodes are ignored.

| Type | Function used here |
| --- | --- |
| 2 / 24 | Inspect fault bits 16–21 and mode bits 22–23; no quantized pose decoding |
| 3 | Enable (control mode only) |
| 4 | Stop; data all zero, so faults are not automatically cleared |
| 17 | Parameter read: index little-endian in bytes 0–1; bytes 2–7 zero |
| 18 | Parameter write: index little-endian in bytes 0–1; value in bytes 4–7 |
| 21 | Inspect little-endian fault/warning bitfields; latch an error if nonzero |

| Register | Encoding | Use |
| --- | --- | --- |
| `0x7005` | uint8 in byte 4 | CSP mode 5 |
| `0x7016` | float32 little-endian | Load-side target angle, radians |
| `0x7017` | float32 little-endian | CSP maximum speed, rad/s |
| `0x7018` | float32 little-endian | Current limit, amperes |
| `0x7019` | float32 little-endian | Load-side measured angle, radians |
| `0x701B` | float32 little-endian | Load-side measured velocity, rad/s |
| `0x7028` | uint32 little-endian | Firmware CAN watchdog; 20000 ticks = 1 second |

Position/velocity use parameter reads rather than the quantized type-2 data:
the vendor manuals/sample revisions have differing velocity/torque scales for
individual models. Reading floats also preserves the reported multiturn angle.
The conversion never silently wraps an angle to `[-pi,pi]`.

## Lifecycle

SocketCAN receive remains nonblocking. Transmit paces accepted frames by at
least 200 microseconds to avoid bursts overflowing a small netdevice TX queue.
Only failed send syscalls reporting ENOBUFS, EAGAIN/EWOULDBLOCK or EINTR are
retried, with a 200-microsecond backoff and a 5-millisecond per-frame deadline.
Accepted frames are never replayed; short sends and permanent errors fail
immediately. A timeout propagates to the existing latched-fault/stop path.
No long software command queue is introduced. Timing still depends on Linux
scheduling; the control-period/feedback checks and firmware watchdog remain
in force. See [Linux send(2)](https://man7.org/linux/man-pages/man2/send.2.html).

The first control write checks elapsed steady-clock time since activation.
Controller-manager may include the initialization handshake in its first or
second reported period. An oversized reported period is tolerated only while
still within one configured maximum control period since activation, with
fresh feedback. That cycle resends held targets with no target advancement.
A delayed first write still faults. Outside this bounded startup window,
the normal configured period limit is enforced without exceptions.

Monitoring opens the socket, queries position/velocity, and exports state
interfaces. It sends no enable, stop, parameter-write or calibration command.

Control activation validates actual encoder positions, stops each configured
motor, selects CSP, and writes/read-checks speed, current and watchdog limits.
It refreshes measured positions, preloads matching position targets, enables
motors and checks acknowledgment of Run mode. Targets are resent immediately
after enable. No simulation initial positions are used.

`read()` drains bounded nonblocking traffic and polls float position/velocity
at the configured rate. `write()` validates all commands, enforces joint
position limits and limits target slew to configured joint velocity.
CAN errors, stale position OR velocity, non-finite data, tracking errors, motor
faults, loss of Run mode or an excessive control-loop period latch an error.
The driver attempts stop frames for every motor it enabled; it never
automatically enables a motor again after a fault. If the bus is broken or
the process is killed, software cannot guarantee delivery of stop frames:
the configured and read-checked firmware CAN watchdog is the remaining
communication-loss mechanism. Stopping/disabling torque does not hold an
unsupported arm against gravity.

This is not hard realtime: Linux scheduling, controller-loop and bus timing
must be validated on the actual machine. Do not treat the current-limit or
watchdog configuration as a substitute for a physical emergency stop.
