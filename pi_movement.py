#!/usr/bin/env python3
"""
pi_movement.py — Laptop 1 (Movement Operator)
Run this first. It owns the serial port and hosts the socket server.

Responsibilities:
  - Serial connection to the Arduino (single owner)
  - WASD hold-to-drive
  - LIDAR (l), E-Stop (e), speed (+/-)
  - TCP socket server on port 9000:
      * Forwards incoming command frames from pi_claw.py to the Arduino
      * Relays all Arduino response frames back to pi_claw.py

Usage:
  python3 pi_movement.py
"""

import struct, serial, time, sys, select, tty, termios
import threading
import socket as sock_mod


# ----------------------------------------------------------------
# SERIAL
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600
_ser     = None
_serial_lock = threading.Lock()   # guards all _ser.write() calls


def openSerial():
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


# ----------------------------------------------------------------
# TPACKET CONSTANTS  (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP         = 0
COMMAND_GET_COLOUR    = 1
COMMAND_FORWARD       = 2
COMMAND_BACKWARD      = 3
COMMAND_TURN_LEFT     = 4
COMMAND_TURN_RIGHT    = 5
COMMAND_STOP_MOTORS   = 6
COMMAND_SET_SPEED     = 7
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
TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)   # 100 bytes
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC      = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1                     # 103 bytes


# ----------------------------------------------------------------
# FRAMING
# ----------------------------------------------------------------

def computeChecksum(data: bytes) -> int:
    r = 0
    for b in data:
        r ^= b
    return r


def packFrame(packetType, command, data=b'', params=None):
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    pkt = struct.pack(TPACKET_FMT, packetType, command, data_padded, *params)
    return MAGIC + pkt + bytes([computeChecksum(pkt)])


def unpackTPacket(raw):
    f = struct.unpack(TPACKET_FMT, raw)
    return {'packetType': f[0], 'command': f[1], 'data': f[2], 'params': list(f[3:])}


def receiveFrame():
    """Read from serial until a valid framed packet arrives. Returns dict or None."""
    MAGIC_HI, MAGIC_LO = MAGIC[0], MAGIC[1]
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
        cs = _ser.read(1)
        if not cs:
            return None
        if cs[0] != computeChecksum(raw):
            continue
        return unpackTPacket(raw)


def sendCommand(cmdType, data=b'', params=None):
    """Thread-safe write to Arduino serial."""
    frame = packFrame(PACKET_TYPE_COMMAND, cmdType, data=data, params=params)
    with _serial_lock:
        _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


# ----------------------------------------------------------------
# PACKET DISPLAY  +  relay to claw operator
# ----------------------------------------------------------------

_claw_conn  = None
_claw_lock  = threading.Lock()


def _relay_to_claw(pkt):
    """Re-frame an Arduino response and forward it to the connected claw client."""
    with _claw_lock:
        if _claw_conn is None:
            return
        try:
            frame = packFrame(pkt['packetType'], pkt['command'],
                              pkt['data'].rstrip(b'\x00'), pkt['params'])
            _claw_conn.sendall(frame)
        except Exception:
            pass


def printPacket(pkt):
    global _estop_state
    ptype, cmd = pkt['packetType'], pkt['command']
    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("\r Response: OK")
        elif cmd == RESP_STATUS:
            _estop_state = pkt['params'][0]
            label = "RUNNING" if _estop_state == STATE_RUNNING else "STOPPED"
            print(f"\r Status: {label}")
        elif cmd == RESP_COLOUR_DATA:
            r, g, b = pkt['params'][0], pkt['params'][1], pkt['params'][2]
            print(f"\r Color  R:{r}  G:{g}  B:{b}  Hz")
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"\r Arduino: {debug}")
    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"\r Arduino: {msg}")


# ----------------------------------------------------------------
# SOCKET SERVER  — runs in a daemon thread
# ----------------------------------------------------------------

SOCKET_PORT = 9000


def _socket_server():
    """
    Accepts exactly one connection from pi_claw.py.
    Forwards received 103-byte command frames directly to the Arduino serial port.
    Arduino responses are relayed back via _relay_to_claw() in the main loop.
    """
    global _claw_conn

    srv = sock_mod.socket(sock_mod.AF_INET, sock_mod.SOCK_STREAM)
    srv.setsockopt(sock_mod.SOL_SOCKET, sock_mod.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', SOCKET_PORT))
    srv.listen(1)
    print(f"\r [Socket] Waiting for claw operator on port {SOCKET_PORT}...")

    while True:
        conn, addr = srv.accept()
        print(f"\r [Socket] Claw operator connected from {addr}")
        with _claw_lock:
            _claw_conn = conn

        buf = b''
        try:
            while True:
                data = conn.recv(256)
                if not data:
                    break
                buf += data
                # Forward complete frames to Arduino as they arrive
                while len(buf) >= FRAME_SIZE:
                    frame = buf[:FRAME_SIZE]
                    buf   = buf[FRAME_SIZE:]
                    with _serial_lock:
                        _ser.write(frame)
        except Exception:
            pass

        with _claw_lock:
            _claw_conn = None
        conn.close()
        print("\r [Socket] Claw operator disconnected. Waiting for reconnect...")


# ----------------------------------------------------------------
# MOTOR + SENSOR HANDLERS
# ----------------------------------------------------------------

_motor_speed = 200

DRIVE_KEYS = {
    'w': COMMAND_FORWARD,
    's': COMMAND_BACKWARD,
    'a': COMMAND_TURN_LEFT,
    'd': COMMAND_TURN_RIGHT,
}


def handleSpeedChange(increase: bool):
    global _motor_speed
    _motor_speed = min(_motor_speed + 20, 255) if increase else max(_motor_speed - 20, 0)
    params = [_motor_speed] + [0] * (PARAMS_COUNT - 1)
    sendCommand(COMMAND_SET_SPEED, params=params)
    print(f"\r Motor speed: {_motor_speed}")


def _handle_tap(ch):
    if ch == 'e':
        sendCommand(COMMAND_ESTOP, data=b'estop')
        print("\r E-Stop toggled.")
    elif ch == 'l':
        if _estop_state == STATE_STOPPED:
            print("\r Refused: E-Stop active.")
        else:
            try:
                from lidar_example_cli_plot import plot_single_scan
                plot_single_scan()
            except ImportError as e:
                print(f"\r LIDAR import failed: {e}")
    elif ch == '+':
        handleSpeedChange(True)
    elif ch == '-':
        handleSpeedChange(False)
    elif ch == ' ':
        sendCommand(COMMAND_STOP_MOTORS)
        print("\r Motors stopped.")
    else:
        print(f"\r Unknown key '{ch}'. Valid: e l w s a d [space] + - q")


# ----------------------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------------------

def runCommandInterface():
    print("=== Laptop 1 — Movement operator ===")
    print("Hold W/A/S/D: drive   l: LIDAR   e: E-Stop   +/-: speed   q: quit\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_drive_key = None

    try:
        tty.setraw(fd)

        while True:
            # --- Receive from Arduino and relay to claw client ---
            if _ser.in_waiting >= FRAME_SIZE:
                pkt = receiveFrame()
                if pkt:
                    printPacket(pkt)
                    _relay_to_claw(pkt)

            # --- Keyboard input (80 ms window) ---
            r, _, _ = select.select([sys.stdin], [], [], 0.08)

            if not r:
                if last_drive_key is not None:
                    sendCommand(COMMAND_STOP_MOTORS)
                    last_drive_key = None
                continue

            ch = sys.stdin.read(1).lower()

            if ch in ('q', '\x03'):  # q or Ctrl+C
                break
            elif ch in DRIVE_KEYS:
                if _estop_state == STATE_STOPPED:
                    print("\r Refused: E-Stop active.")
                    continue
                if ch != last_drive_key:
                    sendCommand(DRIVE_KEYS[ch])
                    last_drive_key = ch
            else:
                if last_drive_key is not None:
                    sendCommand(COMMAND_STOP_MOTORS)
                    last_drive_key = None
                _handle_tap(ch)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        sendCommand(COMMAND_STOP_MOTORS)
        print("\nMovement operator exiting.")


# ----------------------------------------------------------------
# ENTRY POINT
# ----------------------------------------------------------------

if __name__ == '__main__':
    # Socket server runs in background — accepts claw operator connection
    t = threading.Thread(target=_socket_server, daemon=True)
    t.start()

    openSerial()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        closeSerial()