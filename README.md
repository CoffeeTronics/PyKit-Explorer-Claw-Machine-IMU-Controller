# PyKit Claw Machine IMU Controller

BLE central controller for the claw machine project. Reads orientation from a BNO085 IMU and transmits zero-referenced pitch/roll/yaw data over BLE to a peripheral device.

## Target BLE Device

This controller scans for and connects to a peripheral named:

```
CLAW_RX_1292
```

## Hardware

- **Board**: PyKit Explorer
- **IMU**: BNO085 (using game rotation vector - gyro + accel fusion, no magnetometer)
- **BLE Module**: RNBD451
- **User Button**: Pin D3 (active-low with pull-up)
- **Display**: ST7789 240x135 TFT LCD

## State Machine

### States

| State | Description |
|-------|-------------|
| `INITIALIZATION` | Initialize LCD, BLE module, IMU, and button |
| `SCANNING_FOR_CLAW` | Scan for target BLE peripheral (max 3 attempts) |
| `CONNECTING_BLE` | Connect to target and wait for STREAM_OPEN |
| `BLE_CONNECTED` | Connection confirmed, display success message |
| `CALIBRATE_ZERO` | Wait for user button press to set zero point |
| `SEND_ZERO_POSITION` | Send "Move to Home Position" command to Claw |
| `START_IMU_TX` | Display transmission starting message |
| `SEND_IMU_DATA` | Main loop - transmit zero-referenced IMU data |
| `BLE_DISCONNECTED` | Handle disconnect, attempt reconnection |
| `HALTED` | Fatal error - requires manual reset |

### State Transitions

```
INITIALIZATION
      |
      v
SCANNING_FOR_CLAW <----+
      |                |
      | (found)        | (disconnect)
      v                |
CONNECTING_BLE         |
      |                |
      | (STREAM_OPEN)  |
      v                |
BLE_CONNECTED          |
      |                |
      v                |
CALIBRATE_ZERO <-------+------+
      |                       |
      | (button press)        | (re-calibration)
      v                       |
SEND_ZERO_POSITION            |
      |                       |
      v                       |
START_IMU_TX                  |
      |                       |
      v                       |
SEND_IMU_DATA ----------------+
      |
      | (BLE error)
      v
BLE_DISCONNECTED --> SCANNING_FOR_CLAW
```

### LCD Messages

| State | Background | Text Color | Message |
|-------|------------|------------|---------|
| INITIALIZATION | Black | White | "Initializing..." |
| SCANNING_FOR_CLAW | Black | White | "Searching for / target / peripheral..." |
| CONNECTING_BLE | Black | White | "Connecting to / Target / Peripheral..." |
| BLE_CONNECTED | Blue | Yellow | "BLE Connected" |
| CALIBRATE_ZERO (waiting) | Black | White | "Press User Button / to Calibrate" |
| CALIBRATE_ZERO (calibrating) | White | Red | "Calibrating Zero / Position. / Do Not Move" |
| START_IMU_TX | Black | White | "Starting IMU / Transmission" |
| SEND_IMU_DATA | Black | White | Live pitch/roll/yaw values |
| BLE_DISCONNECTED | Black | White | "BLE Connection / Lost - Retrying / to Connect" |
| HALTED (scan fail) | Black | White | "Target Peripheral / Not Found, / Reset PyKit" |
| HALTED (connect fail) | Black | White | "Connect to Claw / Failed, / Reset PyKit" |

## BLE Message Format

### Transmitted to Claw

**Home position command** (sent once after calibration):
```
Move to Home Position\n
```

**IMU data** (sent every 200ms):
```
{pitch} {roll} {yaw}\n
```
Example: `"12.3 -5.6 45.2\n"`

All values are zero-referenced (relative to calibration point). Yaw is normalized to -180 to +180 range.

## Error Recovery

- **IMU errors**: After 5 consecutive read errors (OSError or KeyError), the game rotation vector feature is re-enabled to recover from sensor reset
- **BLE disconnect**: Automatically returns to scanning state and attempts reconnection
- **Scan failure**: After 3 failed scan attempts, halts and displays error message

## Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `LOOP_DT` | 0.2s | Main loop period |
| `MAX_SCAN_ATTEMPTS` | 3 | Scan retries before halt |
| `STREAM_OPEN_TIMEOUT` | 10.0s | Wait time for STREAM_OPEN |
| `BLE_CONNECTED_DISPLAY_TIME` | 2.0s | Duration of "BLE Connected" message |
| `IMU_START_DISPLAY_TIME` | 3.0s | Duration of "Starting IMU" message |
