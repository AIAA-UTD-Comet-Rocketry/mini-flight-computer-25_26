# CanAero Telemetry Protocol

This document describes the current CANaerospace telemetry protocol used by the flight computer.

## Summary
- Transport: Classic CAN (8-byte payload)
- Encoding: CANaerospace header (4 bytes) + 4-byte payload
- Telemetry cadence: chosen by FSM state (1 Hz on ground, 10 Hz in flight)
- Packetization: one packed telemetry packet split across 6 frames

## Frame Map

Status set (sent every tick at the cadence chosen by FSM state):

Datum / Packet            | DTC     | Frame ID(s)       | Bytes | Notes
---|---|---|---|---
Telemetry packet chunk 0  | UCHAR4  | 1400              | 4     | bytes 0-3 of packed packet
Telemetry packet chunk 1  | UCHAR4  | 1401              | 4     | bytes 4-7
Telemetry packet chunk 2  | UCHAR4  | 1402              | 4     | bytes 8-11
Telemetry packet chunk 3  | UCHAR4  | 1403              | 4     | bytes 12-15
Telemetry packet chunk 4  | UCHAR4  | 1404              | 4     | bytes 16-19
Telemetry packet chunk 5  | UCHAR4  | 1405              | 4     | bytes 20-23

## Packed Telemetry Payload (24 bytes, little-endian)

Byte range | Type     | Field                | Scaling / Notes
---|---|---|---
0-3  | uint32 | time_ms            | ms since boot
4-5  | int16  | altitude_ft        | ft
6-7  | int16  | vert_vel_fps_x10   | ft/s * 10
8-9  | int16  | accel_x_mg         | g * 1000 (mg)
10-11| int16  | accel_y_mg         | g * 1000 (mg)
12-13| int16  | accel_z_mg         | g * 1000 (mg)
14-15| int16  | pitch_deg          | degrees
16-17| int16  | roll_deg           | degrees
18-19| int16  | yaw_deg            | degrees
20   | uint8  | fsm_state          | State enum value
21   | uint8  | status_flags       | bitmap below
22   | uint8  | pyro_status        | mirrors gPyroStatus
23   | uint8  | reserved           | 0

## Status Flags (status_flags byte)
- bit 0 SD_LOGGING_ACTIVE - sd_logger_is_active() returned true
- bit 1 MAG_CAL_VALID - NVS load succeeded at boot
- bit 2 EKF_LOCKED - EKF cal phase complete
- bit 3 GROUND_PRESSURE_VALID - baro ground cal succeeded
- bit 4 ARMED - FSM not in IDLE/DISARM
- bit 5..7 reserved (zero)

## Event Frames (unchanged)

ID 1310, DTC=UCHAR2
- byte 4: event_type
- byte 5: event_data

event_type | name           | data
---|---|---
0x01 | BOOT          | reset reason (esp_reset_reason() cast to u8)
0x02 | ARMED         | 0
0x03 | DISARMED      | 0
0x04 | LAUNCH        | peak gTotalAcc rounded to u8 (g)
0x05 | BURNOUT       | 0
0x06 | APOGEE        | 0
0x07 | DROGUE_FIRED  | 0
0x08 | MAIN_FIRED    | 0
0x09 | LANDED        | 0
0x10 | SENSOR_FAIL   | sensor ID (1=IMU, 2=MAG, 3=BARO)
0x11 | SD_FAIL       | 0
0xFF | GENERIC_ERROR | error code

## Notes
- Reassemble chunks in order 0..5 into a 24-byte buffer before decoding.
- All payload fields are little-endian.