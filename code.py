# SPDX-FileCopyrightText: 2024 Microchip Technology Inc.
# SPDX-License-Identifier: MIT

"""
Central board with IMU tilt control
Reads pitch/roll/yaw from BNO085 using game rotation vector and sends over BLE
"""
import pykit_explorer
import time
import board
import busio
import digitalio
import math
from rnbd451 import RNBD451, RNBD451Error
from imu_sensor import IMUSensor

import supervisor
# disable autoreload to prevent VS Code from resetting the board
supervisor.runtime.autoreload = False

# ── Target peripheral name (must match peripheral) ────────────────────────────
TARGET_NAME = "CLAW_RX_1292"

# ── Signal processing constants ───────────────────────────────────────────────
DEADZONE  = 5.0   # magnitude (deg) below which the board is treated as flat
LOOP_DT   = 0.2   # loop period in seconds (must match time.sleep at the bottom)

# ── Hardware setup ────────────────────────────────────────────────────────────
reset_pin = digitalio.DigitalInOut(board.BLE_CLR)
reset_pin.direction = digitalio.Direction.OUTPUT
reset_pin.value = True  # inactive (high)

uart = busio.UART(
    board.BLE_TX,
    board.BLE_RX,
    baudrate=115200,
    timeout=0.1,
)

ble = RNBD451(uart, reset_pin=reset_pin)
imu = IMUSensor()

# Enable game rotation vector for euler_angles_game (gyro + accel fusion, no magnetometer)
imu.enable_game_rotation_vector()

# ── Initialise BLE ────────────────────────────────────────────────────────────
print("[CENTRAL] Hard-resetting module...")
ble.hard_reset(delay=0.1, settle=2.0)
print("[CENTRAL] Reset complete")

ble.enter_command_mode()
print("[CENTRAL] Entered command mode")
print("[CENTRAL] Firmware:", ble.get_firmware_version())

# Ensure transparent UART service is enabled (needed for data mode pipe)
ble.set_default_services(transparent_uart=True)

# Use "No Input No Output with Bonding" so the pair auto-bonds without user interaction
ble.set_pairing_mode(mode=0)

# ── Scan for peripheral ───────────────────────────────────────────────────────
print(f"[CENTRAL] Scanning for '{TARGET_NAME}'...")
target = None

for attempt in range(3):
    devices = ble.scan(interval_ms=100, window_ms=80)
    print(f"[CENTRAL]   Found {len(devices)} device(s):")
    for dev in devices:
        print(f"           {dev['address']} ({dev['name']!r}) RSSI={dev['rssi']} dBm")
        if dev["name"] == TARGET_NAME:
            target = dev
            break
    if target:
        break
    print("[CENTRAL]   Target not found, retrying...")
    time.sleep(1)

if target is None:
    raise RuntimeError(
        f"Could not find '{TARGET_NAME}' after scanning. "
        "Check peripheral is powered and advertising."
    )

print(f"[CENTRAL] Target found: {target['address']} (type={target['addr_type']})")

# ── Connect ───────────────────────────────────────────────────────────────────
print("[CENTRAL] Connecting...")
ble.connect(target["address"], target["addr_type"], timeout=15.0)
print(f"[CENTRAL] Connected to {ble.peer_address}")

# Wait for the transparent UART stream to open
ble.exit_command_mode()
print("[CENTRAL] Back in data mode - waiting for STREAM_OPEN...")
if not ble.wait_for_stream_open(timeout=10.0):
    print("[CENTRAL] STREAM_OPEN not received (may still work on some firmware versions)")

# ── IMU tilt loop ─────────────────────────────────────────────────────────────
print("[CENTRAL] Starting IMU tilt transmission. Press Ctrl-C to stop.")

while True:
    # ── 1. Read pitch, roll, yaw from BNO085 game rotation vector ─────────────
    try:
        roll, pitch, yaw = imu.euler_angles_game
    except OSError as e:
        print(f"[CENTRAL] IMU read error: {e}, skipping...")
        time.sleep(LOOP_DT)
        continue

    # ── 2. Deadzone - suppress transmission when near-flat ────────────────────
    magnitude = math.sqrt(pitch**2 + roll**2)
    if magnitude >= DEADZONE:
        msg = f"{pitch:.1f} {roll:.1f} {yaw:.1f}\n"
    else:
        msg = "0.0 0.0 0.0\n"  # arm holds position

    # ── 3. Transmit ───────────────────────────────────────────────────────────
    print(f"[CENTRAL] TX: {msg.strip()}  (roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f})")
    ble.write(msg.encode())

    # Check for any response
    rx = ble.read_available()
    if rx:
        print(f"[CENTRAL] RX: {rx.decode().strip()}")

    time.sleep(LOOP_DT)
