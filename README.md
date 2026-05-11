# ESP32 Mini Flight Computer 25-26

Firmware for comet rocketry's flight computer built on the ESP32-S3 platform. The system handles sensor fusion, barometric altimetry, flight state management, pyrotechnic deployment, SD card logging, and CAN bus telemetry.

---

## Prerequisites

- ESP-IDF v5.5.x — [Official Installation Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
- VSCode with the ESP-IDF extension (recommended) or any IDE with C/C++ support
- ESP32-S3 board to flash firmware

Build and flash:
```
idf.py build
idf.py erase-flash
idf.py flash monitor
```

---

## Hardware

**Microcontroller:** ESP32-S3

### Sensors (I2C)

| Sensor | Bus | Address | Function |
|--------|-----|---------|----------|
| LSM6DSV80X | I2C0 (SDA=39, SCL=40) | 0x6A | 6-axis IMU — ±80g accelerometer + gyroscope |
| LPS22DF | I2C1 (SDA=41, SCL=42) | 0x5C | Barometric pressure / temperature |
| IIS2MDC | I2C1 (SDA=41, SCL=42) | 0x1E | 3-axis magnetometer |

### GPIO Pinout

| Function | GPIO |
|----------|------|
| Pyro — Drogue 1 (35g CO2) | 6 |
| Pyro — Drogue 2 (45g CO2) | 7 |
| Pyro — Main 1 | 8 |
| Pyro — Main 2 | 9 |
| LED — Status | 1 |
| LED — CAN TX | 2 |
| LED — CAN RX | 4 |
| LED — SD TX | 10 |
| LED — SD RX | 11 |
| CAN RX (TWAI) | 36 |
| CAN TX (TWAI) | 37 |
| IMU INT1 | 38 |
| Baro INT | 46 |
| SD CLK | 34 |
| SD CMD | 47 |
| SD DATA0-3 | 48, 35, 21, 26 |

---

## Software Architecture

The firmware runs on FreeRTOS with five concurrent tasks at 100 Hz each. All sensor data is funnelled into a shared `FusedPacket_t` struct protected by a spinlock.

```
app_main()
│
├── BSP init (I2C, sensors, CAN, GPIO)
├── IMU calibration (NVS-backed, auto-sequence on first run)
├── Ground pressure calibration (1 s average)
│
├── Task: vImuHandlerTask    priority 2 — reads LSM6DSV80X, runs Madgwick AHRS
├── Task: vAltHandlerTask    priority 2 — reads LPS22DF, IIR filters pressure, computes altitude
├── Task: vFsmTask           priority 2 — runs flight state machine at 10 ms
├── Task: vSdLoggerTask      priority 1 — writes CSV to SD card at 100 ms
├── Task: LED_Task           priority 0 — drives LED blink patterns
├── Task: Pyro_Task          priority 4 — fires pyro channels on task notification
└── Task: CAN-TLM            priority 1 — broadcasts CANaerospace telemetry
```

### Sensor Pipeline

```
LSM6DSV80X (raw mg, mdps)
  └─► apply NVS calibration (offset + sensitivity)
  └─► axis remap (BOARD_AXIS_ALIGNMENT)
  └─► FusionBias (gyro bias estimation)
  └─► Madgwick AHRS (quaternion → Euler angles, earth-frame acceleration)
  └─► sensor_velocity_predict() (accel integration → vertical velocity)
  └─► FusedPacket_t { currAcc, currGyro, orientation, linearAcc, gTotalAcc }

LPS22DF (raw hPa)
  └─► hardware LPF at ODR/9 + 64-sample averaging (on-chip)
  └─► sensor_pressure_filter() — 2nd-order Butterworth IIR, fc=5 Hz
  └─► sensor_get_altitude() — ISA pressure-altitude formula (ft MSL)
  └─► sensor_velocity_correct() — complementary filter blends baro Δalt/Δt with accel velocity
  └─► FusedPacket_t { currPress, currTempF, gAltitude, gVerticalVelocity }
```

Calibration parameters (accel offset, accel sensitivity, gyro offset) are stored in NVS and loaded on boot. Re-running calibration requires enabling the `calibration_run_menu()` call in `app_main()`.

---

## Flight State Machine

The FSM runs in `vFsmTask` and progresses through the following states. All thresholds reference `gTotalAcc` (g), `gAltitude` (ft AGL), and `gVerticalVelocity` (ft/s).

```
IDLE ──► ARMED
          │
          │  gTotalAcc > 1.6 g  for 100 ms
          ▼
       BURNING
          │
          │  gTotalAcc < 1.3 g  OR  burn time > 4000 ms
          ▼
        RISING
          │
          │  3 consecutive descending velocity/altitude samples (500 ms period)
          ▼
        APOGEE ──► fires Drogue 1, then Drogue 2 (1000 ms delay)
          │
          │
          ▼
   DROGUE_DESCENT
          │
          │  gAltitude < 1500 ft → fires Main 1
          │  gAltitude < 1300 ft → fires Main 2 (backup)
          ▼
   MAIN_DESCENT
          │
          │  altitude change < 3 ft  OR  velocity < 8 ft/s  for 10 samples
          ▼
        LANDED
```

State transitions and pyro events are broadcast over CAN as event frames.

---

## SD Card Logging

Logs are written as CSV files on a FAT32 SD card mounted at `/sdcard` (4-bit SDMMC, up to 20 MHz).

**CSV columns:**

| Column | Unit |
|--------|------|
| timestamp_s | seconds since boot |
| acc_x, acc_y, acc_z | g (calibrated, body-frame) |
| accel_g | total acceleration magnitude (g) |
| velocity_fps | vertical velocity (ft/s) |
| yaw_deg, pitch_deg, roll_deg | degrees |
| pressure_hpa | raw barometric pressure |
| altitude_ft | AGL altitude (ft) |
| temp_f | temperature (°F) |
| flight_state | FSM state enum value |
| drogue1, drogue2, main1, main2 | pyro fired flags (0/1) |

Log entries are written at 10 Hz. Files are synced every 10 writes to limit data loss on power failure.

---

## CAN Bus Telemetry (CANaerospace)

The system transmits on a 500 kbit/s CAN bus using the CANaerospace protocol (node ID 1). The following message IDs are broadcast at 1 Hz during flight:

| CAN ID | Content | Unit |
|--------|---------|------|
| 304–306 | Accel X/Y/Z | g |
| 320–322 | Pitch / Roll / Yaw | deg |
| 605 | Pressure altitude | ft |
| 1300 | FSM state | enum |
| 1301 | Status flags | bitfield |
| 1302 | Pyro fired status | bitfield |
| 1303 | Total acceleration | g |
| 1304 | Vertical velocity | ft/s |
| 1310 | Event frames (launch, apogee, pyro, etc.) | — |

Status flag bits (ID 1301): `SD_LOGGING`, `MAG_CAL_VALID`, `EKF_LOCKED`, `GROUND_PRESS_VALID`, `ARMED`.

---

## Libraries

### Custom Components (`components/`)

**FlightFSM**
State machine implementation. `FlightStateAdapter.c` defines entry/exit callbacks and transition conditions for all 9 flight states. Pyro channels are fired via `xTaskNotify` to `Pyro_Task` from state transition callbacks.

**CanAero**
CANaerospace protocol encoder/transmitter. Manages a pool of 8 TX frame slots returned via TWAI TX-done callbacks. `can_telemetry.cpp` drives a 1 Hz broadcast task; `canaerospace.cpp` handles frame construction and big-endian byte ordering per spec.

**Fusion** (Seb Madgwick)
Madgwick AHRS algorithm. Estimates 3D orientation as a quaternion from gyroscope, accelerometer, and optional magnetometer data. Features configurable acceleration and magnetic rejection, automatic bias estimation (`FusionBias`), and axis remapping (`FusionRemap`) to align the sensor board frame to the rocket body frame.

**IIS2MDC / LPS22DF / LSM6DSV80X**
ST Microelectronics sensor driver libraries. Provide register-level read/write abstraction over the BSP I2C wrappers. Drivers are integrated via callback function pointers (`ReadReg`/`WriteReg`) registered during `bsp_init()`.

### Managed Components (`managed_components/`)

**espressif/esp-dsp** v1.8.1
Espressif DSP library. Used for the `dsps_biquad_f32` biquad IIR filter applied to the raw barometric pressure. On ESP32-S3 the filter resolves to the hardware-accelerated AES3 vector variant. Coefficients are a 2nd-order Butterworth low-pass at fc = 5 Hz / fs = 100 Hz, computed with `scipy.signal.butter`.

### ESP-IDF Components (REQUIRES)

| Component | Purpose |
|-----------|---------|
| `esp_driver_gpio` | LED and pyro GPIO output control |
| `esp_driver_i2c` | I2C master bus for sensor communication |
| `esp_driver_twai` | CAN bus (TWAI) transmit/receive |
| `esp_driver_sdmmc` | SD card host controller |
| `esp_timer` | High-resolution timestamps (`sensor_get_tick_ms`) |
| `fatfs` | FAT32 filesystem for SD card log files |
| `nvs_flash` | Non-volatile storage for IMU calibration parameters |
| `espcoredump` | Panic-time core dump to UART/Flash for crash diagnosis |
