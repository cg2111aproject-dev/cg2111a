#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data
  - Hold W/A/S/D to drive like a game character (tty raw mode)

Packet framing format (103 bytes total):
  MAGIC (2 B) | TPacket (100 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""

import struct
import serial
import time
import sys
import select
import tty        # <-- NEW
import termios    # <-- NEW
import alex_camera

from second_terminal import relay

# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    #relay.checkSecondTerminal(_ser)
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP      = 0
COMMAND_GET_COLOUR = 1
COMMAND_FORWARD      = 2
COMMAND_BACKWARD     = 3
COMMAND_TURN_LEFT    = 4
COMMAND_TURN_RIGHT   = 5
COMMAND_STOP_MOTORS  = 6
COMMAND_SET_SPEED    = 7
COMMAND_CLAW_BASE     = 8
COMMAND_CLAW_SHOULDER = 9
COMMAND_CLAW_ELBOW    = 10
COMMAND_CLAW_GRIPPER  = 11
COMMAND_CLAW_HOME     = 12
COMMAND_CLAW_SPEED    = 13

RESP_OK          = 0
RESP_STATUS      = 1
RESP_COLOUR_DATA = 2

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.
    Returns a packet dict, or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_HI:
            continue

        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            continue

        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """Print a received TPacket in human-readable form."""
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("\r Response: OK                    ")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("\r Status: RUNNING                  ")
            else:
                print("\r Status: STOPPED                  ")
        elif cmd == RESP_COLOUR_DATA:
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"\r\nColor:")
            print(f"\r  Red={r} Hz")
            print(f"\r  Green={g} Hz")
            print(f"\r  Blue={b} Hz\n")

        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"\r Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"\r Arduino: {msg}")
    else:
        print(f"\r Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    if _estop_state == STATE_STOPPED:
        print("\r Refused: E-Stop is active.")
        return
    print("\r Requesting colour...")
    sendCommand(COMMAND_GET_COLOUR)


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

_camera = alex_camera.cameraOpen()
_frames_remaining = 5


def handleCameraCommand():
    global _frames_remaining
    if _estop_state == STATE_STOPPED:
        print("\r Refused: E-Stop is active.")
    elif _frames_remaining == 0:
        print("\r No more frames remaining.")
    else:
        _greyscaleframe = alex_camera.captureGreyscaleFrame(_camera)
        alex_camera.renderGreyscaleFrame(_greyscaleframe)
        _frames_remaining -= 1
        print(f"\r Frames remaining: {_frames_remaining}")


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------

from lidar_example_cli_plot import plot_single_scan

def handleLidarCommand():
    if _estop_state == STATE_STOPPED:
        print("\r Refused: E-Stop is active.")
    else:
        plot_single_scan()


# ----------------------------------------------------------------
# MOTOR CONTROL
# ----------------------------------------------------------------

_motor_speed = 200

# Maps WASD keys to Arduino drive commands
DRIVE_KEYS = {                       # <-- NEW: used by runCommandInterface
    'w': COMMAND_FORWARD,
    's': COMMAND_BACKWARD,
    'a': COMMAND_TURN_LEFT,
    'd': COMMAND_TURN_RIGHT,
}

def handleSpeedChange(increase: bool):
    global _motor_speed
    if increase:
        _motor_speed = min(_motor_speed + 20, 255)
    else:
        _motor_speed = max(_motor_speed - 20, 0)
    params = [_motor_speed] + [0] * (PARAMS_COUNT - 1)
    sendCommand(COMMAND_SET_SPEED, params=params)
    print(f"\r Motor speed set to {_motor_speed}")


# ----------------------------------------------------------------
# CLAW CONTROL
# ----------------------------------------------------------------

def handleClawCommand(line):
    if line == 'h':
        sendCommand(COMMAND_CLAW_HOME)
        print("\r Claw homing.")
        return

    prompt_map = {
        'i': (COMMAND_CLAW_BASE,     "Base angle (0-180)"),
        'o': (COMMAND_CLAW_SHOULDER, "Shoulder angle (50-135)"),
        'k': (COMMAND_CLAW_ELBOW,    "Elbow angle (105-180)"),
        'n': (COMMAND_CLAW_GRIPPER,  "Gripper angle (90-105)"),
        'v': (COMMAND_CLAW_SPEED,    "Claw speed ms-per-degree (1-999)"),
    }
    cmd_type, prompt = prompt_map[line]

    # Temporarily restore normal terminal mode so input() works
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    try:
        val = int(input(f"\r  {prompt}: ").strip())
    except ValueError:
        print("\r Invalid value.")
        return
    finally:
        tty.setraw(fd)   # back to raw mode after input

    params = [val] + [0] * (PARAMS_COUNT - 1)
    sendCommand(cmd_type, params=params)


# ----------------------------------------------------------------
# SINGLE-TAP HANDLER  <-- NEW: replaces handleUserInput for non-drive keys
# ----------------------------------------------------------------

def _handle_tap(ch):
    """Dispatch a single keypress for non-drive commands."""
    if ch == 'e':
        print("\r Sending E-Stop...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')
    elif ch == 'c':
        handleColorCommand()
    elif ch == 'p':
        handleCameraCommand()
    elif ch == 'l':
        handleLidarCommand()
    elif ch == '+':
        handleSpeedChange(increase=True)
    elif ch == '-':
        handleSpeedChange(increase=False)
    elif ch == ' ':
        sendCommand(COMMAND_STOP_MOTORS)
        print("\r Motors stopped.")
    elif ch in ('i', 'o', 'k', 'n', 'h', 'v'):
        handleClawCommand(ch)
    else:
        print(f"\r Unknown key: '{ch}'. Valid: e c p l w s a d [space] + - i o k n h v q")


# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE  <-- REPLACED with raw-tty hold-to-drive
# ----------------------------------------------------------------

def runCommandInterface():
    """
    Main loop using raw terminal mode so WASD drive the robot
    while held — no Enter needed, like a game character.

    Key repeat (fired by the OS while a key is held) keeps
    re-sending the drive command every ~20 ms.  When the key is
    released, select() times out after 80 ms and stops the motors.
    """
    print("Hold W/A/S/D to drive. Tap E/C/P/L/+/-/etc for commands. Q to quit.\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_drive_key = None

    try:
        tty.setraw(fd)          # raw mode: characters arrive immediately, no echo

        while True:
            # --- receive packets from Arduino ---
            if _ser.in_waiting >= FRAME_SIZE:
                pkt = receiveFrame()
                if pkt:
                    printPacket(pkt)
                    relay.onPacketReceived(packFrame(pkt['packetType'], pkt['command'],pkt['data'], pkt['params']))

            # --- check for a keypress (80 ms window) ---
            r, _, _ = select.select([sys.stdin], [], [], 0.08)

            if not r:
                # Nothing pressed — stop if we were driving
                if last_drive_key is not None:
                    sendCommand(COMMAND_STOP_MOTORS)
                    last_drive_key = None
                continue

            relay.checkSecondTerminal(_ser)

            ch = sys.stdin.read(1).lower()

            if ch == 'q':
                break

            elif ch in DRIVE_KEYS:
                if _estop_state == STATE_STOPPED:
                    print("\r Refused: E-Stop is active.")
                    continue
                # Only send if direction changed (avoids serial spam while held)
                if ch != last_drive_key:
                    sendCommand(DRIVE_KEYS[ch])
                    last_drive_key = ch

            else:
                # Non-drive key: stop motors first, then handle the tap
                if last_drive_key is not None:
                    sendCommand(COMMAND_STOP_MOTORS)
                    last_drive_key = None
                _handle_tap(ch)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # restore terminal
        sendCommand(COMMAND_STOP_MOTORS)   # safety stop on exit


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    relay.start()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        closeSerial()
        alex_camera.cameraClose(_camera)
        relay.shutdown()
