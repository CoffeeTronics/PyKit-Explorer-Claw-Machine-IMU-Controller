# SPDX-FileCopyrightText: 2024 Microchip Technology Inc.
# SPDX-License-Identifier: MIT

"""
PyKit Explorer Claw Machine IMU Controller v2.0
BLE + USART Failover Protocol

Protocol:
  $ - IMU data: $seq,pitch,roll,yaw
  # - Commands: #CMD:USE_BLE, #CMD:USE_USART, #CMD:CALIBRATE, etc.
  ! - Status: !STATUS:BLE_FAIL
  @ - ACK: @ACK:RECEIVED
"""

import time
import board
import busio
import digitalio
import neopixel
import random

import pykit_explorer
from rnbd451 import RNBD451, RNBD451Error
from imu_sensor import IMUSensor
from lcd_display import LCDDisplay, Colors
from digital_io import EdgeDetector
from uart_comms import UARTComms

import supervisor
supervisor.runtime.autoreload = False

# =============================================================================
# Constants
# =============================================================================

# Protocol prefixes
MSG_PREFIX_DATA = '$'
MSG_PREFIX_CMD = '#'
MSG_PREFIX_STATUS = '!'
MSG_PREFIX_ACK = '@'

# State machine states
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
STATE_DROPPING_CLAW = "DROPPING_CLAW"
STATE_ERROR_RECOVERY = "ERROR_RECOVERY"

# Timing constants
TARGET_NAME = "CLAW_RX__1292"
LOOP_DT = 0.1  # 10Hz IMU rate
MAX_SCAN_ATTEMPTS = 3
STREAM_OPEN_TIMEOUT = 10.0
BLE_CONNECTED_DISPLAY_TIME = 2.0
IMU_START_DISPLAY_TIME = 3.0
DROPPING_CLAW_DELAY = 3.0

# Failover timing
ACK_TIMEOUT_MS = 50
DATA_TIMEOUT_SEC = 2.0
PACKET_LOSS_WARNING = 5
PACKET_LOSS_FAILOVER = 20

# Reconnection timing
RECONNECT_CONTINUOUS_SEC = 60
RECONNECT_SLOW_SEC = 120
RECONNECT_SLOW_INTERVAL = 30.0
RECONNECT_RARE_INTERVAL = 300.0
DEVICE_COUNT_THRESHOLD = 200

# Motor control
DEADZONE_DEG = 5.0           # IMU tilt below this = stopped
MAX_TILT_DEG = 45.0          # Full speed at this tilt
MAX_VELOCITY_MTURNS = 5000   # milli-motor-turns/s at max tilt

# Joint-to-node mapping (node IDs match SAME70 CAN config)
JOINT_MAP = {
    'J1': 0,   # Base yaw - controlled by IMU yaw
    'J2': 1,   # Shoulder - controlled by IMU pitch
    'J3': 2,   # Elbow - controlled by IMU pitch (linked to J2)
    'J5': 3,   # Wrist pitch - controlled by IMU roll
}

# Recovery timing
RECOVERY_LEVEL1_WAIT = 0.1
RECOVERY_LEVEL2_WAIT = 2.0
RECOVERY_LEVEL3_WAIT = 2.0
MAX_RECOVERY_ATTEMPTS = 5

# NeoPixel colors
COLOR_BLE = (0, 0, 255)      # Blue for BLE
COLOR_USART = (0, 255, 0)    # Green for USART
COLOR_FAIL = (255, 0, 0)     # Red for failure
COLOR_OFF = (0, 0, 0)

# =============================================================================
# Global State
# =============================================================================

current_state = STATE_INITIALIZATION
state_entry_time = 0.0
scan_attempts = 0
target = None

# IMU calibration
zero_roll = 0.0
zero_pitch = 0.0
zero_yaw = 0.0
imu_error_count = 0
locked_pitch = 0.0
locked_roll = 0.0
locked_yaw = 0.0
drop_command_sent = False

# Protocol state
sequence_number = 0
expected_ack_seq = None
ack_sent_time = 0.0
awaiting_ack = False

# Connection state
active_channel = "BOTH"  # "BLE", "USART", "BOTH"
ble_connected = False
usart_connected = False
last_ble_rx_time = 0.0
last_usart_rx_time = 0.0
missed_packets = 0
total_packets_sent = 0

# Recovery state
recovery_level = 1
recovery_attempts = 0
disconnect_time = 0.0
last_reconnect_attempt = 0.0

# Debug counters
debug_cmd_count = 0
debug_last_cmd_time = 0.0

# Hardware references (initialized later)
ble = None
imu = None
lcd = None
group = None
palette = None
line1 = None
line2 = None
line3 = None
button = None
drop_claw_button = None
reset_pin = None
uart = None
de9_uart = None
pixels = None

# =============================================================================
# LCD Display
# =============================================================================

def update_lcd(text1, text2="", text3="", bg_color=Colors.BLACK, text_color=Colors.WHITE):
    global group, palette, line1, line2, line3
    palette[0] = bg_color
    line1.color = text_color
    line2.color = text_color
    line3.color = text_color
    line1.text = text1
    line2.text = text2
    line3.text = text3

# =============================================================================
# NeoPixel Indication
# =============================================================================

def update_neopixel():
    global pixels, active_channel, ble_connected, usart_connected

    if not ble_connected and not usart_connected:
        pixels[0] = COLOR_FAIL
    elif active_channel == "USART" or (active_channel == "BOTH" and not ble_connected):
        pixels[0] = COLOR_USART
    elif ble_connected:
        pixels[0] = COLOR_BLE
    else:
        pixels[0] = COLOR_OFF

# =============================================================================
# State Machine
# =============================================================================

def enter_state(new_state):
    global current_state, state_entry_time, sequence_number
    current_state = new_state
    state_entry_time = time.monotonic()
    print("[STATE] Entering " + new_state)

    # Reset sequence on connection-related state changes
    if new_state in [STATE_BLE_CONNECTED, STATE_CALIBRATE_ZERO]:
        sequence_number = 0

# =============================================================================
# Protocol Functions
# =============================================================================

def send_imu_data(pitch, roll, yaw, channel="BOTH"):
    """Send IMU data packet with sequence number"""
    global sequence_number, total_packets_sent

    msg = MSG_PREFIX_DATA + str(sequence_number) + "," + \
          str(round(pitch, 1)) + "," + \
          str(round(roll, 1)) + "," + \
          str(round(yaw, 1)) + "\n"

    if channel in ["BLE", "BOTH"] and ble_connected:
        try:
            ble.write(msg.encode())
            print("[BLE_TX] " + msg.strip())
        except Exception as e:
            print("[BLE_ERROR] Write failed: " + str(e))

    if channel in ["USART", "BOTH"]:
        de9_uart.send(msg)
        print("[UART_TX] " + msg.strip())

    sequence_number = (sequence_number + 1) & 0xFF
    total_packets_sent += 1

def send_command(cmd, arg="", channel="BOTH", require_ack=True):
    """Send a command and optionally wait for ACK"""
    global awaiting_ack, ack_sent_time, expected_ack_seq

    if arg:
        msg = MSG_PREFIX_CMD + cmd + ":" + arg + "\n"
    else:
        msg = MSG_PREFIX_CMD + cmd + "\n"

    if channel in ["BLE", "BOTH"] and ble_connected:
        try:
            ble.write(msg.encode())
            print("[BLE_TX] " + msg.strip())
        except Exception as e:
            print("[BLE_ERROR] Write failed: " + str(e))

    if channel in ["USART", "BOTH"]:
        de9_uart.send(msg)
        print("[UART_TX] " + msg.strip())

    if require_ack:
        awaiting_ack = True
        ack_sent_time = time.monotonic()
        return wait_for_ack()

    return True

def send_status(status, channel="BOTH"):
    """Send a status message"""
    msg = MSG_PREFIX_STATUS + "STATUS:" + status + "\n"

    if channel in ["BLE", "BOTH"] and ble_connected:
        try:
            ble.write(msg.encode())
            print("[BLE_TX] " + msg.strip())
        except Exception as e:
            print("[BLE_ERROR] Write failed")

    if channel in ["USART", "BOTH"]:
        de9_uart.send(msg)
        print("[UART_TX] " + msg.strip())

def wait_for_ack(timeout_ms=ACK_TIMEOUT_MS):
    """Non-blocking ACK wait with timeout"""
    global awaiting_ack

    start = time.monotonic()
    timeout_sec = timeout_ms / 1000.0

    while (time.monotonic() - start) < timeout_sec:
        # Check BLE
        if ble_connected:
            try:
                rx = ble.read_available()
                if rx:
                    line = rx.decode().strip()
                    print("[BLE_RX] " + line)
                    if line.startswith(MSG_PREFIX_ACK):
                        awaiting_ack = False
                        return True
            except:
                pass

        # Check USART
        de9_rx = de9_uart.receive(64)
        if de9_rx:
            line = de9_rx.strip()
            print("[UART_RX] " + line)
            if line.startswith(MSG_PREFIX_ACK):
                awaiting_ack = False
                return True

        time.sleep(0.001)

    print("[ERROR] ACK timeout")
    awaiting_ack = False
    return False

def send_sync():
    """Send SYNC handshake and wait for ACK with matching nonce"""
    global sequence_number

    nonce = random.randint(0, 255)
    msg = MSG_PREFIX_CMD + "CMD:SYNC," + str(nonce) + "\n"

    de9_uart.send(msg)
    print("[UART_TX] " + msg.strip())

    # Wait for ACK:SYNC,<nonce>
    start = time.monotonic()
    expected = "ACK:SYNC," + str(nonce)

    while (time.monotonic() - start) < 0.05:
        de9_rx = de9_uart.receive(64)
        if de9_rx:
            line = de9_rx.strip()
            print("[UART_RX] " + line)
            if expected in line:
                sequence_number = 0
                print("[STATE] SYNC complete, seq reset")
                return True
        time.sleep(0.001)

    print("[ERROR] SYNC ACK timeout")
    return False

def check_incoming_data():
    """Check for and process incoming data on both channels"""
    global last_ble_rx_time, last_usart_rx_time, ble_connected

    # Check BLE
    if ble_connected:
        try:
            rx = ble.read_available()
            if rx:
                last_ble_rx_time = time.monotonic()
                line = rx.decode().strip()
                print("[BLE_RX] " + line)
                process_received_line(line, "BLE")
        except Exception as e:
            print("[BLE_ERROR] Read failed: " + str(e))
            ble_connected = False

    # Check USART
    de9_rx = de9_uart.receive(64)
    if de9_rx:
        last_usart_rx_time = time.monotonic()
        line = de9_rx.strip()
        print("[UART_RX] " + line)
        process_received_line(line, "USART")

def process_received_line(line, source):
    """Process a received line from either channel"""
    global usart_connected

    if source == "USART":
        usart_connected = True

    if line.startswith(MSG_PREFIX_ACK):
        print("[" + source + "_STATUS] ACK: " + line[1:])
    elif line.startswith(MSG_PREFIX_STATUS):
        print("[" + source + "_STATUS] Remote: " + line[1:])
    elif line.startswith(MSG_PREFIX_CMD):
        print("[" + source + "_STATUS] Command: " + line[1:])


# =============================================================================
# Motor Control Functions
# =============================================================================

def angle_to_velocity(angle_deg):
    """Convert IMU angle to motor velocity (milli-turns/s)"""
    if abs(angle_deg) < DEADZONE_DEG:
        return 0
    # Linear scaling from deadzone to max
    sign = 1 if angle_deg > 0 else -1
    magnitude = abs(angle_deg) - DEADZONE_DEG
    scaled = (magnitude / (MAX_TILT_DEG - DEADZONE_DEG)) * MAX_VELOCITY_MTURNS
    return int(sign * min(scaled, MAX_VELOCITY_MTURNS))

def send_both(msg):
    """Send message on both BLE and USART channels"""
    global ble_connected, debug_cmd_count, debug_last_cmd_time
    debug_cmd_count += 1
    now = time.monotonic()
    # Print every 10th command or if more than 2 sec since last print
    if debug_cmd_count % 10 == 1 or (now - debug_last_cmd_time) > 2.0:
        print("[DEBUG] cmd #" + str(debug_cmd_count) + " BLE=" + str(ble_connected) + " msg=" + msg.strip()[:30])
        debug_last_cmd_time = now
    if ble_connected:
        try:
            ble.write(msg.encode())
        except Exception as e:
            print("[BLE_ERROR] Write #" + str(debug_cmd_count) + " failed: " + str(e))
            ble_connected = False
    de9_uart.send(msg)

def send_motor_commands(pitch, roll, yaw):
    """Send velocity commands based on IMU orientation.
    
    Always sends on BOTH channels (BLE + USART) for redundancy.
    If BLE fails, USART continues uninterrupted.
    """
    global debug_cmd_count
    # Yaw -> J1 (base rotation)
    v_j1 = angle_to_velocity(yaw)
    v_j2 = angle_to_velocity(pitch)
    v_j3 = angle_to_velocity(pitch)
    v_j5 = angle_to_velocity(roll)
    
    if debug_cmd_count % 50 == 0:
        print("[MOTOR] p=" + str(round(pitch,1)) + " r=" + str(round(roll,1)) + " y=" + str(round(yaw,1)))
        print("[VEL] J1=" + str(v_j1) + " J2=" + str(v_j2) + " J3=" + str(v_j3) + " J5=" + str(v_j5))
    
    # Pitch -> J2 + J3 (shoulder + elbow, linked)
    v_j2 = angle_to_velocity(pitch)
    v_j3 = angle_to_velocity(pitch)  # Same as J2, linked
    
    # Roll -> J5 (wrist pitch)
    v_j5 = angle_to_velocity(roll)
    
    # Build and send commands on BOTH channels
    for joint, vel in [('J1', v_j1), ('J2', v_j2), ('J3', v_j3), ('J5', v_j5)]:
        node = JOINT_MAP[joint]
        msg = "V," + str(node) + "," + str(vel) + "\n"
        send_both(msg)

def stop_all_motors():
    """Send STOP to all motors on both channels"""
    msg = "STOP\n"
    send_both(msg)
    print("[MOTOR] Stopped all motors")

def arm_and_home_motors():
    """Arm all joints and move to HOME position"""
    print("[MOTOR] Arming and homing all joints")
    
    # Arm all joints for velocity control
    for joint, node in JOINT_MAP.items():
        msg = "GO," + str(node) + "\n"
        send_both(msg)
        print("[MOTOR] Armed " + joint + " (node " + str(node) + ")")
        time.sleep(0.1)  # Small delay between arming
    
    # Move all joints to HOME (zero position)
    for joint, node in JOINT_MAP.items():
        msg = "N," + str(node) + ",0\n"
        send_both(msg)
        print("[MOTOR] Homing " + joint)
        time.sleep(0.05)
    
    print("[MOTOR] All joints armed and moving to HOME")
# =============================================================================
# Failover Logic
# =============================================================================

def switch_to_usart():
    """Switch to USART-only mode"""
    global active_channel

    print("[STATE] Switching to USART mode")
    active_channel = "USART"

    if send_command("CMD", "USE_USART", channel="USART"):
        send_status("BLE_FAIL", channel="USART")
        update_neopixel()
        return True
    return False

def switch_to_ble():
    """Switch to BLE mode"""
    global active_channel

    print("[STATE] Switching to BLE mode")
    active_channel = "BLE"

    if send_command("CMD", "USE_BLE"):
        send_status("BLE_OK")
        update_neopixel()
        return True
    return False

def switch_to_both():
    """Switch to both channels mode"""
    global active_channel

    print("[STATE] Switching to BOTH mode")
    active_channel = "BOTH"

    send_command("CMD", "USE_BOTH")
    update_neopixel()

def check_ble_health():
    """Check if BLE connection is healthy"""
    global ble_connected

    if not ble_connected:
        return False

    # Check for %DISCONNECT% or connection timeout
    try:
        rx = ble.read_available()
        if rx:
            data = rx.decode()
            if "%DISCONNECT%" in data:
                print("[BLE_STATUS] Disconnected")
                ble_connected = False
                return False
    except:
        ble_connected = False
        return False

    return True

def should_attempt_reconnect():
    """Determine if we should attempt BLE reconnection"""
    global disconnect_time, last_reconnect_attempt

    now = time.monotonic()
    elapsed = now - disconnect_time

    if elapsed < RECONNECT_CONTINUOUS_SEC:
        return True  # Continuous attempts
    elif elapsed < RECONNECT_SLOW_SEC:
        if (now - last_reconnect_attempt) >= RECONNECT_SLOW_INTERVAL:
            return True
    else:
        if (now - last_reconnect_attempt) >= RECONNECT_RARE_INTERVAL:
            return True

    return False

def count_nearby_devices():
    """Count nearby BLE devices to determine RF congestion"""
    try:
        devices = ble.scan(interval_ms=50, window_ms=40)
        return len(devices)
    except:
        return 0

# =============================================================================
# Recovery Functions
# =============================================================================

def recovery_level1_soft_flush():
    """Level 1: Soft flush - clear buffers"""
    print("[RECOVERY] Level 1: Soft flush")

    # Drain UART buffers
    while uart.in_waiting:
        uart.read(uart.in_waiting)

    # Clear DE9 buffer
    while True:
        data = de9_uart.receive(64)
        if not data:
            break

    time.sleep(RECOVERY_LEVEL1_WAIT)

def recovery_level2_warm_reset():
    """Level 2: Warm reset - reinit peripherals"""
    global ble

    print("[RECOVERY] Level 2: Warm reset")

    # Toggle BLE module briefly
    reset_pin.value = False
    time.sleep(0.05)
    reset_pin.value = True
    time.sleep(0.5)

    # Drain any boot messages
    while uart.in_waiting:
        uart.read(uart.in_waiting)

    time.sleep(RECOVERY_LEVEL2_WAIT)

def recovery_level3_hard_reboot():
    """Level 3: Hard reboot - full BLE reset"""
    global ble, ble_connected

    print("[RECOVERY] Level 3: Hard reboot")

    ble.hard_reset(delay=0.1, settle=2.0)

    # Drain boot messages
    while uart.in_waiting:
        data = uart.read(uart.in_waiting)
        print("[BLE_RX] ", data)

    ble_connected = False

    # Re-init BLE
    ble.enter_command_mode()
    ble.set_default_services(transparent_uart=True)
    ble.set_pairing_mode(mode=0)
    ble.reboot()
    time.sleep(2.0)

    while uart.in_waiting:
        uart.read(uart.in_waiting)

    # Send SYNC
    send_sync()

    time.sleep(RECOVERY_LEVEL3_WAIT)

def run_recovery():
    """Run escalating recovery sequence"""
    global recovery_level, recovery_attempts

    if recovery_attempts >= MAX_RECOVERY_ATTEMPTS:
        print("[RECOVERY] All attempts failed")
        return False

    recovery_attempts += 1

    if recovery_level == 1:
        recovery_level1_soft_flush()
        if recovery_attempts >= 2:
            recovery_level = 2
    elif recovery_level == 2:
        recovery_level2_warm_reset()
        if recovery_attempts >= 4:
            recovery_level = 3
    else:
        recovery_level3_hard_reboot()

    return True

# =============================================================================
# State Handlers
# =============================================================================

def handle_initialization():
    global ble, imu, lcd, group, palette, line1, line2, line3
    global button, drop_claw_button, reset_pin, uart, de9_uart, pixels

    # LCD init
    lcd = LCDDisplay()
    lcd.backlight_on()
    group, palette = lcd.make_group(Colors.BLACK)
    line1 = lcd.add_label(group, "Initializing...", 120, 40, color=Colors.WHITE, scale=2)
    line2 = lcd.add_label(group, "", 120, 65, color=Colors.WHITE, scale=2)
    line3 = lcd.add_label(group, "", 120, 90, color=Colors.WHITE, scale=2)

    # NeoPixel init
    pixels = neopixel.NeoPixel(board.NEOPIXEL, 5, brightness=0.3)
    pixels.fill(COLOR_OFF)

    # BLE module init
    reset_pin = digitalio.DigitalInOut(board.BLE_CLR)
    reset_pin.direction = digitalio.Direction.OUTPUT
    reset_pin.value = True

    uart = busio.UART(board.BLE_TX, board.BLE_RX, baudrate=115200, timeout=0.1)
    ble = RNBD451(uart, reset_pin=reset_pin)

    # DE9 UART init
    de9_uart = UARTComms(tx=board.DEBUG_TX, rx=board.DEBUG_RX, baudrate=115200, timeout=0.1)
    print("[CENTRAL] DE9 UART initialized")

    # IMU init
    imu = IMUSensor()
    imu.enable_game_rotation_vector()

    # Button init
    button = EdgeDetector(board.D3)
    drop_claw_button = EdgeDetector(board.D5)

    # BLE hard reset
    print("[CENTRAL] Hard-resetting module...")
    ble.hard_reset(delay=0.1, settle=2.0)
    print("[CENTRAL] Reset complete")

    ble.enter_command_mode()
    print("[CENTRAL] Entered command mode")
    print("[CENTRAL] Firmware:", ble.get_firmware_version())

    ble.set_default_services(transparent_uart=True)
    ble.set_pairing_mode(mode=0)
    print("[CENTRAL] Rebooting to apply settings...")
    ble.reboot()
    time.sleep(2.0)
    print("[CENTRAL] Reboot complete")

    while uart.in_waiting:
        data = uart.read(uart.in_waiting)
        print("[CENTRAL] Drained:", data)
    time.sleep(0.5)

    print("[CENTRAL] Re-entering command mode...")
    ble._in_command_mode = False
    ble.enter_command_mode()
    print("[CENTRAL] In command mode:", ble.in_command_mode)
    time.sleep(0.3)

    print("[CENTRAL] Ready to scan")
    enter_state(STATE_SCANNING_FOR_CLAW)

def handle_scanning():
    global scan_attempts, target

    update_lcd("Searching for", "target", "peripheral...")

    print("[CENTRAL] Scanning attempt " + str(scan_attempts + 1))

    # Drain UART and add delay before scan
    while uart.in_waiting:
        uart.read(uart.in_waiting)
    time.sleep(0.3)

    try:
        devices = ble.scan(interval_ms=100, window_ms=80)
    except RNBD451Error as e:
        print("[CENTRAL] Scan error: " + str(e))
        scan_attempts += 1
        if scan_attempts >= MAX_SCAN_ATTEMPTS:
            update_lcd("Scan Failed", "Reset PyKit")
            enter_state(STATE_HALTED)
        else:
            print("[CENTRAL] Retrying scan...")
            time.sleep(1)
        return

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
    global target, ble_connected

    update_lcd("Connecting to", "Target", "Peripheral...")

    try:
        print("[CENTRAL] Connecting...")
        ble.connect(target["address"], target["addr_type"], timeout=15.0)
        print("[CENTRAL] Connected to " + str(ble.peer_address))
        
        # Add delay before exiting command mode to let connection stabilize
        print("[CENTRAL] Waiting 500ms for connection to stabilize...")
        time.sleep(0.5)
        
        print("[CENTRAL] Exiting command mode...")
        ble.exit_command_mode()
        print("[CENTRAL] Command mode exited, waiting for STREAM_OPEN...")

        # Drain any buffered data first
        while uart.in_waiting:
            raw = uart.read(uart.in_waiting)
            print("[CENTRAL] Drained before STREAM_OPEN: " + repr(raw))
        
        if ble.wait_for_stream_open(timeout=STREAM_OPEN_TIMEOUT):
            print("[CENTRAL] STREAM_OPEN received")
            ble_connected = True
            enter_state(STATE_BLE_CONNECTED)
        else:
            print("[CENTRAL] STREAM_OPEN not received after " + str(STREAM_OPEN_TIMEOUT) + "s")
            # Try to see what we DID receive
            if uart.in_waiting:
                raw = uart.read(uart.in_waiting)
                print("[CENTRAL] Received instead: " + repr(raw))
            update_lcd("Connect to Claw", "Failed,", "Reset PyKit")
            enter_state(STATE_HALTED)
    except RNBD451Error as e:
        print("[CENTRAL] Connection error: " + str(e))
        update_lcd("Connect to Claw", "Failed,", "Reset PyKit")
        enter_state(STATE_HALTED)

def handle_ble_connected():
    global active_channel

    update_lcd("BLE Connected", bg_color=Colors.BLUE, text_color=Colors.YELLOW)
    update_neopixel()

    # Send channel mode to receiver
    send_command("CMD", "USE_BOTH", require_ack=False)
    active_channel = "BOTH"

    if time.monotonic() - state_entry_time > BLE_CONNECTED_DISPLAY_TIME:
        enter_state(STATE_CALIBRATE_ZERO)

def handle_calibrate_zero():
    global zero_roll, zero_pitch, zero_yaw

    if time.monotonic() - state_entry_time < 0.3:
        update_lcd("Press User Button", "to Calibrate")

        # Send calibration command to SAME70
        send_command("CMD", "CALIBRATE", require_ack=False)

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
    """Arm all joints and move to HOME position before IMU streaming"""
    print("[CENTRAL] Arming motors and moving to HOME position")
    
    # Arm and home all motors (sends on both BLE + USART)
    arm_and_home_motors()
    
    # Give motors time to reach home position
    time.sleep(1.0)
    
    enter_state(STATE_START_IMU_TX)

def handle_start_imu_tx():
    if time.monotonic() - state_entry_time < 0.3:
        update_lcd("Starting IMU", "Transmission")

    if time.monotonic() - state_entry_time > IMU_START_DISPLAY_TIME:
        enter_state(STATE_SEND_IMU_DATA)

# Stale IMU detection
last_imu_values = (None, None, None)
stale_imu_count = 0
STALE_THRESHOLD = 20  # Reset IMU if values unchanged for this many reads

def handle_send_imu_data():
    global imu_error_count, locked_pitch, locked_roll, locked_yaw
    global ble_connected, disconnect_time
    global last_imu_values, stale_imu_count

    button.update()
    drop_claw_button.update()

    # Check BLE health
    if not check_ble_health() and ble_connected:
        ble_connected = False
        disconnect_time = time.monotonic()
        print("[BLE_STATUS] Connection lost, switching to USART")
        switch_to_usart()

    if button.fell:
        print("[CENTRAL] Re-calibration requested")
        enter_state(STATE_CALIBRATE_ZERO)
        return

    try:
        roll, pitch, yaw = imu.euler_angles_game
        imu_error_count = 0

        # Check for stale data (IMU frozen)
        current_values = (round(roll, 1), round(pitch, 1), round(yaw, 1))
        if current_values == last_imu_values:
            stale_imu_count += 1
            if stale_imu_count >= STALE_THRESHOLD:
                print("[IMU] Stale data detected, resetting IMU")
                try:
                    imu.soft_reset()
                    time.sleep(0.1)
                    imu.enable_game_rotation_vector()
                except Exception as e:
                    print("[IMU] Reset failed:", e)
                stale_imu_count = 0
                return  # Skip this cycle, fresh data next time
        else:
            stale_imu_count = 0
        last_imu_values = current_values

    except (OSError, KeyError, RuntimeError) as e:
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
    # Normalize all angles to -180 to +180 range
    if rel_pitch > 180:
        rel_pitch -= 360
    elif rel_pitch < -180:
        rel_pitch += 360
    if rel_roll > 180:
        rel_roll -= 360
    elif rel_roll < -180:
        rel_roll += 360
    if rel_yaw > 180:
        rel_yaw -= 360
    elif rel_yaw < -180:
        rel_yaw += 360

    if drop_claw_button.fell:
        print("[CENTRAL] Drop claw requested")
        locked_pitch = rel_pitch
        locked_roll = rel_roll
        locked_yaw = rel_yaw
        enter_state(STATE_DROPPING_CLAW)
        return

    # Display values
    p = str(round(rel_pitch, 1))
    r = str(round(rel_roll, 1))
    y = str(round(rel_yaw, 1))
    update_lcd("Pitch: " + p, "Roll:  " + r, "Yaw:   " + y)

    # Send IMU data
    send_motor_commands(rel_pitch, rel_roll, rel_yaw)

    # Check for incoming data
    check_incoming_data()

    # Update LED
    update_neopixel()

def handle_dropping_claw():
    global drop_command_sent, ble_connected

    if time.monotonic() - state_entry_time < 0.3:
        update_lcd("Dropping Claw", bg_color=Colors.GREEN, text_color=Colors.YELLOW)
        if not drop_command_sent:
            print("[CENTRAL] Sending Drop Claw")
            send_command("CMD", "DROP_CLAW", require_ack=False)
            drop_command_sent = True

    p = str(round(locked_pitch, 1))
    r = str(round(locked_roll, 1))
    y = str(round(locked_yaw, 1))

    # Send locked position
    send_motor_commands(locked_pitch, locked_roll, locked_yaw)

    # Check for incoming data
    check_incoming_data()

    if time.monotonic() - state_entry_time > DROPPING_CLAW_DELAY:
        print("[CENTRAL] Claw drop complete, returning to calibration")
        drop_command_sent = False
        enter_state(STATE_CALIBRATE_ZERO)

def handle_ble_disconnected():
    global scan_attempts, ble_connected, last_reconnect_attempt, disconnect_time

    update_lcd("BLE Connection", "Lost - Using", "USART Backup", bg_color=Colors.GREEN)
    update_neopixel()

    # Check if we should attempt reconnection
    if should_attempt_reconnect():
        # Check device count
        device_count = count_nearby_devices()
        print("[CENTRAL] Nearby devices: " + str(device_count))

        if device_count < DEVICE_COUNT_THRESHOLD:
            last_reconnect_attempt = time.monotonic()
            print("[CENTRAL] Attempting BLE reconnect...")
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
        else:
            print("[CENTRAL] Too many devices, skipping reconnect")

    # Continue sending IMU data over USART while waiting
    try:
        roll, pitch, yaw = imu.euler_angles_game
        rel_roll = roll - zero_roll
        rel_pitch = pitch - zero_pitch
        rel_yaw = yaw - zero_yaw
        if rel_yaw > 180:
            rel_yaw -= 360
        elif rel_yaw < -180:
            rel_yaw += 360

        send_motor_commands(rel_pitch, rel_roll, rel_yaw)
        check_incoming_data()
    except:
        pass

def handle_error_recovery():
    global recovery_level, recovery_attempts

    update_lcd("Error Recovery", "Level " + str(recovery_level), "Attempt " + str(recovery_attempts + 1),
               bg_color=Colors.RED, text_color=Colors.WHITE)

    if run_recovery():
        # Check if recovery succeeded
        if ble_connected or usart_connected:
            print("[RECOVERY] Success!")
            recovery_level = 1
            recovery_attempts = 0
            enter_state(STATE_SEND_IMU_DATA)
        else:
            print("[RECOVERY] Attempt " + str(recovery_attempts) + " failed")
    else:
        # All recovery failed
        update_lcd("COMMS FAIL", "System Halted", "", bg_color=Colors.RED, text_color=Colors.BLACK)
        enter_state(STATE_HALTED)

def handle_halted():
    # Flash red LED
    if int(time.monotonic() * 4) % 2:
        pixels[0] = COLOR_FAIL
    else:
        pixels[0] = COLOR_OFF

# =============================================================================
# State Handler Map
# =============================================================================

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
    STATE_DROPPING_CLAW: handle_dropping_claw,
    STATE_ERROR_RECOVERY: handle_error_recovery,
}

# =============================================================================
# Main Loop
# =============================================================================

print("[CENTRAL] Starting state machine v2.0...")
print("[CENTRAL] Protocol: $=data, #=cmd, !=status, @=ack")

while True:
    handler = STATE_HANDLERS.get(current_state)
    if handler:
        handler()
    else:
        print("[ERROR] Unknown state: " + current_state)
        break
    time.sleep(LOOP_DT)
