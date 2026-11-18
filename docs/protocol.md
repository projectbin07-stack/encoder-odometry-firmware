# Serial protocol, version 1

The firmware and the host exchange binary frames over the board's serial port
(UART / USB-UART bridge). Nothing else is sent on that port: no text, no log
lines. The only exception is the ESP32 ROM bootloader's boot message, which
the host discards as a framing error.

Implementations:

- firmware: [`lib/odocore/src/odocore/protocol.cpp`](../lib/odocore/src/odocore/protocol.cpp),
  [`framing.cpp`](../lib/odocore/src/odocore/framing.cpp)
- host: [`host/src/odobridge/protocol.py`](../host/src/odobridge/protocol.py)

Both are tested against the same byte vectors in
[`test/vectors/protocol_vectors.json`](../test/vectors/protocol_vectors.json)
(C++: `test/test_protocol_vectors`, Python: `host/tests/test_protocol.py`).

## Framing

```
wire frame = COBS(packet) 0x00
packet     = version:u8  type:u8  seq:u8  length:u8  body[length]  crc:u16
```

| Field   | Size | Meaning |
|---------|------|---------|
| version | 1    | Protocol version, currently `1`. A receiver rejects any other value. |
| type    | 1    | Message type, table below. |
| seq     | 1    | Sender's sequence number, incremented per frame, wraps 255 -> 0. Each direction has its own counter. The host counts gaps to detect lost frames. |
| length  | 1    | Body length in bytes. Must equal the fixed size for `type`. |
| body    | n    | Message fields, little-endian. |
| crc     | 2    | CRC-16/CCITT-FALSE over `version .. body`, sent low byte first. |

- **COBS** (Consistent Overhead Byte Stuffing) removes every `0x00` from the
  packet, so `0x00` can mark the end of a frame. Overhead: 1 byte per 254, plus
  1. A receiver that joins mid-stream resynchronises at the next `0x00`.
- **CRC-16/CCITT-FALSE**: polynomial `0x1021`, initial value `0xFFFF`, no input
  or output reflection, no final XOR. Check value: `"123456789"` -> `0x29B1`.
- **Numbers**: integers are little-endian two's complement; floats are IEEE-754
  binary32, little-endian.
- **Size limits**: the largest body is 60 bytes (WHEEL_STATE), the largest
  packet 66 bytes, the largest frame 68 bytes. Receivers drop anything longer
  than 72 bytes between delimiters as a framing error.
- A packet is accepted only if the COBS decoding, CRC, version, type and length
  are all valid. There is no partial decoding and no acknowledgement except
  for parameter requests.

Error accounting on the firmware side (reported in DIAGNOSTICS):
`rx_crc_errors` counts CRC mismatches; `rx_frame_errors` counts everything
else (bad COBS, overlong frame, too short, unknown version, unknown type,
length mismatch).

## Message types

| Type | Name             | Direction       | Body bytes | Rate |
|------|------------------|-----------------|-----------:|------|
| 0x01 | VELOCITY_COMMAND | host -> firmware | 12 | as sent; must repeat faster than the watchdog timeout |
| 0x02 | WHEEL_STATE      | firmware -> host | 60 | every `telemetry_divider` control periods (default 5 -> 20 Hz) |
| 0x03 | ODOMETRY         | firmware -> host | 28 | with WHEEL_STATE, and in reply to SET_POSE |
| 0x04 | DIAGNOSTICS      | firmware -> host | 41 | every 10 x `telemetry_divider` periods (default 2 Hz) |
| 0x05 | PARAM_GET        | host -> firmware | 1  | on request |
| 0x06 | PARAM_SET        | host -> firmware | 5  | on request |
| 0x07 | PARAM_VALUE      | firmware -> host | 6  | reply to every PARAM_GET / PARAM_SET |
| 0x08 | SET_POSE         | host -> firmware | 12 | on request; all zeros resets odometry |

Messages of a firmware -> host type received by the firmware are ignored.

### 0x01 VELOCITY_COMMAND (12 bytes)

| Offset | Type | Field | Unit |
|-------:|------|-------|------|
| 0 | f32 | vx | m/s, forward |
| 4 | f32 | vy | m/s, left (ignored on a differential drive) |
| 8 | f32 | wz | rad/s, counter-clockwise |

Body frame per ROS REP-103. The firmware runs inverse kinematics, scales all
four wheel targets by the same factor if any exceeds `max_wheel_speed` (the
path is kept, only slowed), and feeds the command watchdog. A command with a
NaN or infinite component is ignored and does not feed the watchdog.

If no valid command arrives for `watchdog_ms` (default 500 ms), all targets go
to zero, the PI controllers are reset and the motor outputs go to zero duty
(L298N: brake; DRV: brake, by default) until the next command. Each such
event increments `watchdog_trips`.

### 0x02 WHEEL_STATE (60 bytes)

| Offset | Type | Field |
|-------:|------|-------|
| 0 | u32 | t_us: firmware `micros()` at the control step |
| 4 + 14*i | i32 | count[i]: accumulated 4x encoder count (wraps modulo 2^32) |
| 8 + 14*i | f32 | velocity[i]: measured wheel speed, rad/s |
| 12 + 14*i | f32 | target[i]: wheel speed set-point, rad/s |
| 16 + 14*i | i16 | duty[i]: motor duty x 10000, -10000 .. 10000 |

`i` = 0..3 for FL, FR, RL, RR (fields are interleaved per wheel).

### 0x03 ODOMETRY (28 bytes)

| Offset | Type | Field | Unit |
|-------:|------|-------|------|
| 0  | u32 | t_us | firmware `micros()` |
| 4  | f32 | x | m |
| 8  | f32 | y | m |
| 12 | f32 | theta | rad, wrapped to (-pi, pi] |
| 16 | f32 | vx | m/s, from measured wheel speeds |
| 20 | f32 | vy | m/s |
| 24 | f32 | wz | rad/s |

The firmware accumulates the pose in double precision; the frame carries it
as float32 (about 0.1 mm resolution at 1 km from the origin).

### 0x04 DIAGNOSTICS (41 bytes)

| Offset | Type | Field |
|-------:|------|-------|
| 0  | u32 | t_us |
| 4  | u32 x 4 | illegal_transitions[FL, FR, RL, RR]: quadrature transitions with both channels changed (missed edges). Always 0 with hardware counters. |
| 20 | u32 | loop_overruns: control periods skipped or that ran longer than the period |
| 24 | u32 | watchdog_trips |
| 28 | u32 | max_loop_us: longest control step seen |
| 32 | u32 | rx_crc_errors |
| 36 | u32 | rx_frame_errors |
| 40 | u8  | flags: bit 0 watchdog expired, bit 1 hardware counters in use, bit 2 command saturated |

### 0x05 PARAM_GET (1 byte), 0x06 PARAM_SET (5 bytes), 0x07 PARAM_VALUE (6 bytes)

| Message | Offset | Type | Field |
|---------|-------:|------|-------|
| PARAM_GET   | 0 | u8  | id |
| PARAM_SET   | 0 | u8  | id |
|             | 1 | f32 | value |
| PARAM_VALUE | 0 | u8  | id |
|             | 1 | u8  | status: 0 ok, 1 unknown id, 2 out of range, 3 read-only |
|             | 2 | f32 | value: the current value after the request (0 for an unknown id) |

A rejected PARAM_SET leaves the parameter unchanged. Integer parameters are
rounded to the nearest integer. Changes take effect at the next control step
and are not stored in flash: they are lost at reset.

| Id | Name | Range | Default | Notes |
|---:|------|-------|--------:|-------|
| 1 | kp | 0 .. 10 | 0.02 | duty per rad/s |
| 2 | ki | 0 .. 100 | 0.2 | duty per rad |
| 3 | kd | 0 .. 1 | 0 | duty per rad/s^2, on measurement |
| 4 | kff | 0 .. 1 | 0.03 | feed-forward, duty per rad/s |
| 5 | ks | 0 .. 1 | 0 | static-friction feed-forward, duty |
| 6 | output_limit | 0 .. 1 | 1 | duty ceiling |
| 7 | slew_rate | 0 .. 1000 | 4 | duty per second, 0 = off |
| 8 | d_cutoff_hz | 0 .. 1000 | 20 | derivative low-pass, 0 = off |
| 9 | max_wheel_speed | 0.1 .. 1000 | 25 | rad/s |
| 10 | watchdog_ms | 20 .. 10000 | 500 | integer |
| 11 | wheel_radius | 0.001 .. 1 | 0.05 | m |
| 12 | half_length | 0.001 .. 5 | 0.15 | m (lx) |
| 13 | half_width | 0.001 .. 5 | 0.17 | m (ly) |
| 14 | roller_angle_deg | 1 .. 89 | 45 | mecanum only |
| 15 | ticks_per_wheel_rev | 1 .. 1e7 | 38000 | 4x counts per wheel revolution |
| 16 | drive_type | 0 .. 1 | 0 | 0 mecanum, 1 differential; integer |
| 17 | telemetry_divider | 1 .. 1000 | 5 | integer |
| 18 | estimator_low_cps | 0 .. 1e6 | 200 | counts/s; below: period estimate only |
| 19 | estimator_high_cps | 0 .. 1e6 | 800 | counts/s; above: fixed-period only |
| 20 | zero_timeout_ms | 1 .. 10000 | 250 | no edge for this long reads as stopped; integer |
| 21 | control_rate_hz | read-only | 100 | |
| 22 | protocol_version | read-only | 1 | |

### 0x08 SET_POSE (12 bytes)

| Offset | Type | Field |
|-------:|------|-------|
| 0 | f32 | x, m |
| 4 | f32 | y, m |
| 8 | f32 | theta, rad |

The firmware replies with an ODOMETRY frame showing the new pose.

## Worked example

VELOCITY_COMMAND with vx = 0.5 m/s, vy = 0, wz = 0, seq 0 (the first entry of
the shared vectors):

```
packet: 01 01 00 0c | 00 00 00 3f  00 00 00 00  00 00 00 00 | 73 0e
        ver typ seq len   vx=0.5f       vy=0          wz=0       crc=0x0e73
frame:  03 01 01 02 0c 01 01 02 3f 01 01 01 01 01 01 01 03 73 0e 00
```

COBS splits the packet at each zero: `[01 01]` -> `03 01 01`, `[0c]` ->
`02 0c`, two empty blocks -> `01 01`, `[3f]` -> `02 3f`, seven more empty
blocks -> `01` x 7, and the final `[73 0e]` -> `03 73 0e`, then the `00`
delimiter.

## Versioning

The version byte changes whenever an existing message's layout or meaning
changes. New message types can be added within a version; receivers count
unknown types as framing errors and carry on. A host should read parameter 22
(`protocol_version`) when it connects and refuse to drive a firmware whose
version it does not know.
