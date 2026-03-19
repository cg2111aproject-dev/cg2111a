#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data

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
import alex_camera


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
# ADDED: motor drive commands
COMMAND_FORWARD      = 2
COMMAND_BACKWARD     = 3
COMMAND_TURN_LEFT    = 4
COMMAND_TURN_RIGHT   = 5
COMMAND_STOP_MOTORS  = 6
COMMAND_SET_SPEED    = 7
# ADDED: claw commands
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

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (padded/truncated to MAX_STR_LEN)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (103)
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

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
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
    """Return True if the E-Stop is currently active (system stopped)."""
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
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")

        elif cmd == RESP_COLOUR_DATA:
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"\nColor: ")
            print(f"Red={r} Hz")
            print(f"Green={g} Hz")
            print(f"Blue={b} Hz\n")

        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    """Request a color reading from the Arduino and display it."""
    if _estop_state == STATE_STOPPED:
        print("Refused: E-Stop is active. Color scan aborted.")
        return
    elif (_estop_state == STATE_RUNNING):
        print("Requesting Colour")
        sendCommand(COMMAND_GET_COLOUR)
    pass


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

_camera = alex_camera.cameraOpen()
_frames_remaining = 5


def handleCameraCommand():
    """Capture and display a greyscale frame."""
    global _frames_remaining
    if _estop_state == STATE_STOPPED:
        print("Refused: E-Stop is active. Camera scan aborted.")

    elif _frames_remaining == 0:
        print("No more frames remaining")

    else:
        _greyscaleframe = alex_camera.captureGreyscaleFrame(_camera)
        alex_camera.renderGreyscaleFrame(_greyscaleframe)
        _frames_remaining -= 1
        print(f"Frames Remaining: {_frames_remaining}")

    return


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------

from lidar_example_cli_plot import plot_single_scan

def handleLidarCommand():
    """Perform a single LIDAR scan and render it."""
    if _estop_state == STATE_STOPPED:
        print("Refused: E-Stop is active. LIDAR scan aborted.")
    elif _estop_state == STATE_RUNNING:
        plot_single_scan()


# ----------------------------------------------------------------
# ADDED: MOTOR CONTROL
# ----------------------------------------------------------------

# Current speed value sent with COMMAND_SET_SPEED (0-255)
_motor_speed = 200

def handleMotorCommand(direction):
    """Send a drive command to the Arduino. Refused if E-Stop is active."""
    if _estop_state == STATE_STOPPED:
        print("Refused: E-Stop is active.")
        return
    command_map = {
        'w': COMMAND_FORWARD,
        's': COMMAND_BACKWARD,
        'a': COMMAND_TURN_LEFT,
        'd': COMMAND_TURN_RIGHT,
        ' ': COMMAND_STOP_MOTORS,
    }
    sendCommand(command_map[direction])

def handleSpeedChange(increase: bool):
    """Increase or decrease motor speed by 20, clamped to 0-255."""
    global _motor_speed
    if increase:
        _motor_speed = min(_motor_speed + 20, 255)
    else:
        _motor_speed = max(_motor_speed - 20, 0)
    params = [_motor_speed] + [0] * (PARAMS_COUNT - 1)
    sendCommand(COMMAND_SET_SPEED, params=params)
    print(f"Motor speed set to {_motor_speed}")


# ----------------------------------------------------------------
# ADDED: CLAW CONTROL
# ----------------------------------------------------------------

def handleClawCommand(line):
    """
    Send a claw servo command to the Arduino.
    Keys:
      i  base         (prompts for angle)
      o  shoulder     (prompts for angle)
      k  elbow        (prompts for angle)
      n  gripper      (prompts for angle)
      h  home all servos
      v  set claw speed (prompts for ms-per-degree)
    """
    if line == 'h':
        sendCommand(COMMAND_CLAW_HOME)
        print("Claw homing.")
        return

    prompt_map = {
        'i': (COMMAND_CLAW_BASE,     "Base angle (0-180)"),
        'o': (COMMAND_CLAW_SHOULDER, "Shoulder angle (50-135)"),
        'k': (COMMAND_CLAW_ELBOW,    "Elbow angle (105-180)"),
        'n': (COMMAND_CLAW_GRIPPER,  "Gripper angle (90-105)"),
        'v': (COMMAND_CLAW_SPEED,    "Claw speed ms-per-degree (1-999)"),
    }
    cmd_type, prompt = prompt_map[line]
    try:
        val = int(input(f"  {prompt}: ").strip())
    except ValueError:
        print("Invalid value.")
        return
    params = [val] + [0] * (PARAMS_COUNT - 1)
    sendCommand(cmd_type, params=params)


# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

def handleUserInput(line):
    """
    Dispatch a single line of user input.

    Keys:
      e        E-Stop toggle
      c        colour sensor
      p        camera
      l        LIDAR
      w/s/a/d  drive forward / backward / left / right   (ADDED)
      [space]  stop motors                                (ADDED)
      +/-      increase / decrease motor speed            (ADDED)
      i/o/k/n  claw base / shoulder / elbow / gripper    (ADDED)
      h        claw home                                  (ADDED)
      v        claw speed                                 (ADDED)
    """
    if line == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')

    elif line == 'c':
        print("Sending colour sensor command")
        handleColorCommand()

    elif line == "p":
        print("Sending Camera command")
        handleCameraCommand()

    elif line == 'l':
        print("Sending LIDAR command")
        handleLidarCommand()

    # ADDED: motor commands
    elif line in ('w', 's', 'a', 'd', ' '):
        handleMotorCommand(line)

    elif line == '+':
        handleSpeedChange(increase=True)

    elif line == '-':
        handleSpeedChange(increase=False)

    # ADDED: claw commands
    elif line in ('i', 'o', 'k', 'n', 'h', 'v'):
        handleClawCommand(line)

    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, l, w, s, a, d, [space], +/-, i, o, k, n, h, v")


def runCommandInterface():
    """
    Main command loop.

    Uses select.select() to simultaneously receive packets from the Arduino
    and read typed user input from stdin without either blocking the other.
    """
    print("Sensor interface ready. Type e / c / p / l / w / s / a / d / [space] / + / - / i / o / k / n / h / v and press Enter.")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)

        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            handleUserInput(line)

        time.sleep(0.05)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        closeSerial()
        alex_camera.cameraClose(_camera)