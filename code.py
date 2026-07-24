# SPDX-FileCopyrightText: 2024 Microchip Technology Inc.
# SPDX-License-Identifier: MIT

import pykit_explorer
import busio
import digitalio
from rnbd451 import RNBD451, RNBD451Error
from imu_sensor import IMUSensor
from lcd_display import LCDDisplay, Colors
from digital_io import EdgeDetector

import supervisor
supervisor.runtime.autoreload = False

STATE_INITIALIZATION = "INITIALIZATION"
STATE_SCANNING_FOR_CLAW = "SCANNING_FOR_CLAW"
STATE_CONNECTING_BLE = "CONNECTING_BLE"
STATE_BLE_CONNECTED = "BLE_CONNECTED"
STATE_CALIBRATE_ZERO = "CALIBRATE_ZERO"
STATE_SEND_ZERO_POSITION = "SEND_ZERO_POSITION"
STATE_START_IMU_TX = "START_IMU_TX"
STATE_SEND_IMU_DATA = "SEND_IMU_DATA"
STATE_BLE_DISCONNECTED = "BLE_DISCONNECTED"
STATE_HALTED = "HALTED"

TARGET_NAME = "CLAW_RX_1292"
LOOP_DT = 0.2
MAX_SCAN_ATTEMPTS = 3
STREAM_OPEN_TIMEOUT = 10.0
BLE_CONNECTED_DISPLAY_TIME = 2.0
IMU_START_DISPLAY_TIME = 3.0

current_state = STATE_INITIALIZATION
scan_attempts = 0
target = None
state_entry_time = 0.0
zero_roll = 0.0
zero_pitch = 0.0
zero_yaw = 0.0
imu_error_count = 0

def update_lcd(text1, text2="", text3="", bg_color=Colors.BLACK, text_color=Colors.WHITE):
    global group, palette, line1, line2, line3
    palette[0] = bg_color
    line1.color = text_color
    line2.color = text_color
    line3.color = text_color
    line1.text = text1
    line2.text = text2
    line3.text = text3

def enter_state(new_state):
    global current_state, state_entry_time
    current_state = new_state
    state_entry_time = time.monotonic()
    print("[STATE] Entering " + new_state)

def handle_initialization():
    global ble, imu, lcd, group, palette, line1, line2, line3, button, reset_pin, uart
    lcd = LCDDisplay()
    lcd.backlight_on()
    group, palette = lcd.make_group(Colors.BLACK)
    line1 = lcd.add_label(group, "Initializing...", 120, 40, color=Colors.WHITE, scale=2)
    line2 = lcd.add_label(group, "", 120, 65, color=Colors.WHITE, scale=2)
    line3 = lcd.add_label(group, "", 120, 90, color=Colors.WHITE, scale=2)
    reset_pin = digitalio.DigitalInOut(board.BLE_CLR)
    reset_pin.direction = digitalio.Direction.OUTPUT
    reset_pin.value = True
    uart = busio.UART(board.BLE_TX, board.BLE_RX, baudrate=115200, timeout=0.1)
    ble = RNBD451(uart, reset_pin=reset_pin)
    imu = IMUSensor()
    imu.enable_game_rotation_vector()
    button = EdgeDetector(board.D3)
    print("[CENTRAL] Hard-resetting module...")
    ble.hard_reset(delay=0.1, settle=2.0)
    print("[CENTRAL] Reset complete")
    ble.enter_command_mode()
    print("[CENTRAL] Entered command mode")
    print("[CENTRAL] Firmware:", ble.get_firmware_version())
    ble.set_default_services(transparent_uart=True)
    ble.set_pairing_mode(mode=0)
    enter_state(STATE_SCANNING_FOR_CLAW)

def handle_scanning():
    global scan_attempts, target
    update_lcd("Searching for", "target", "peripheral...")
    print("[CENTRAL] Scanning attempt " + str(scan_attempts + 1))
    devices = ble.scan(interval_ms=100, window_ms=80)
    print("[CENTRAL] Found " + str(len(devices)) + " devices")
    for dev in devices:
        addr = dev["address"]
        name = dev["name"]
        rssi = dev["rssi"]
        print("  " + addr + " " + repr(name) + " RSSI=" + str(rssi))
        if name == TARGET_NAME:
            target = dev
            print("[CENTRAL] Target found: " + addr)
            enter_state(STATE_CONNECTING_BLE)
            return
    scan_attempts += 1
    if scan_attempts >= MAX_SCAN_ATTEMPTS:
        update_lcd("Target Peripheral", "Not Found,", "Reset PyKit")
        enter_state(STATE_HALTED)
    else:
        print("[CENTRAL] Target not found, retrying...")
        time.sleep(1)

def handle_connecting():
    global target
    update_lcd("Connecting to", "Target", "Peripheral...")
    try:
        print("[CENTRAL] Connecting...")
        ble.connect(target["address"], target["addr_type"], timeout=15.0)
        print("[CENTRAL] Connected to " + str(ble.peer_address))
        ble.exit_command_mode()
        print("[CENTRAL] Waiting for STREAM_OPEN...")
        if ble.wait_for_stream_open(timeout=STREAM_OPEN_TIMEOUT):
            print("[CENTRAL] STREAM_OPEN received")
            enter_state(STATE_BLE_CONNECTED)
        else:
            print("[CENTRAL] STREAM_OPEN not received")
            update_lcd("Connect to Claw", "Failed,", "Reset PyKit")
            enter_state(STATE_HALTED)
    except RNBD451Error as e:
        print("[CENTRAL] Connection error: " + str(e))
        update_lcd("Connect to Claw", "Failed,", "Reset PyKit")
        enter_state(STATE_HALTED)

def handle_ble_connected():
    update_lcd("BLE Connected", bg_color=Colors.BLUE, text_color=Colors.YELLOW)
    if time.monotonic() - state_entry_time > BLE_CONNECTED_DISPLAY_TIME:
        enter_state(STATE_CALIBRATE_ZERO)

def handle_calibrate_zero():
    global zero_roll, zero_pitch, zero_yaw
    if time.monotonic() - state_entry_time < 0.3:
        update_lcd("Press User Button", "to Calibrate")
    button.update()
    if button.fell:
        update_lcd("Calibrating Zero", "Position.", "Do Not Move", bg_color=Colors.WHITE, text_color=Colors.RED)
        try:
            roll, pitch, yaw = imu.euler_angles_game
            zero_roll = roll
            zero_pitch = pitch
            zero_yaw = yaw
            print("[CENTRAL] Zero calibrated")
            time.sleep(1)
            enter_state(STATE_SEND_ZERO_POSITION)
        except OSError as e:
            print("[CENTRAL] IMU error: " + str(e))
            update_lcd("IMU Error", "Try Again")
            time.sleep(1)

def handle_send_zero_position():
    print("[CENTRAL] Sending Move to Home Position")
    ble.write(b"Move to Home Position\n")
    enter_state(STATE_START_IMU_TX)

def handle_start_imu_tx():
    if time.monotonic() - state_entry_time < 0.3:
        update_lcd("Starting IMU", "Transmission")
    if time.monotonic() - state_entry_time > IMU_START_DISPLAY_TIME:
        enter_state(STATE_SEND_IMU_DATA)

def handle_send_imu_data():
    global imu_error_count
    button.update()
    if button.fell:
        print("[CENTRAL] Re-calibration requested")
        enter_state(STATE_CALIBRATE_ZERO)
        return
    try:
        roll, pitch, yaw = imu.euler_angles_game
        imu_error_count = 0
    except (OSError, KeyError) as e:
        imu_error_count += 1
        print("[CENTRAL] IMU error, skipping")
        if imu_error_count > 5:
            print("[CENTRAL] Re-enabling IMU game rotation vector")
            try:
                imu.enable_game_rotation_vector()
            except:
                pass
            imu_error_count = 0
        return
    rel_roll = roll - zero_roll
    rel_pitch = pitch - zero_pitch
    rel_yaw = yaw - zero_yaw
    if rel_yaw > 180:
        rel_yaw -= 360
    elif rel_yaw < -180:
        rel_yaw += 360
    p = str(round(rel_pitch,1))
    r = str(round(rel_roll,1))
    y = str(round(rel_yaw,1))
    update_lcd("Pitch: " + p, "Roll:  " + r, "Yaw:   " + y)
    msg = p + " " + r + " " + y + "\n"
    print("[CENTRAL] TX: " + msg.strip())
    try:
        ble.write(msg.encode())
    except Exception as e:
        print("[CENTRAL] BLE write error")
        enter_state(STATE_BLE_DISCONNECTED)
        return
    try:
        rx = ble.read_available()
        if rx:
            print("[CENTRAL] RX: " + rx.decode().strip())
    except Exception as e:
        print("[CENTRAL] BLE read error")
        enter_state(STATE_BLE_DISCONNECTED)


def handle_ble_disconnected():
    global scan_attempts
    update_lcd("BLE Connection", "Lost - Retrying", "to Connect")
    print("[CENTRAL] BLE disconnected, reconnecting...")
    time.sleep(2)
    scan_attempts = 0
    try:
        ble.enter_command_mode()
        enter_state(STATE_SCANNING_FOR_CLAW)
    except RNBD451Error as e:
        print("[CENTRAL] Re-init BLE module")
        ble.hard_reset(delay=0.1, settle=2.0)
        ble.enter_command_mode()
        ble.set_default_services(transparent_uart=True)
        ble.set_pairing_mode(mode=0)
        enter_state(STATE_SCANNING_FOR_CLAW)

def handle_halted():
    pass

STATE_HANDLERS = {
    STATE_INITIALIZATION: handle_initialization,
    STATE_SCANNING_FOR_CLAW: handle_scanning,
    STATE_CONNECTING_BLE: handle_connecting,
    STATE_BLE_CONNECTED: handle_ble_connected,
    STATE_CALIBRATE_ZERO: handle_calibrate_zero,
    STATE_SEND_ZERO_POSITION: handle_send_zero_position,
    STATE_START_IMU_TX: handle_start_imu_tx,
    STATE_SEND_IMU_DATA: handle_send_imu_data,
    STATE_BLE_DISCONNECTED: handle_ble_disconnected,
    STATE_HALTED: handle_halted,
}

print("[CENTRAL] Starting state machine...")

while True:
    handler = STATE_HANDLERS.get(current_state)
    if handler:
        handler()
    else:
        print("[ERROR] Unknown state: " + current_state)
        break
    time.sleep(LOOP_DT)
