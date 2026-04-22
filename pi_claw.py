#!/usr/bin/env python3
"""
pi_claw.py — Laptop 2 (Claw Operator)
Run this after pi_movement.py is already running.


Responsibilities:
  - Connects to pi_movement.py's socket server on port 9000
  - Sends claw, camera, and colour sensor command frames through the socket
    (pi_movement.py forwards them to the Arduino over serial)
  - Receives Arduino response frames relayed back from pi_movement.py
  - Hold i/o/k/n (or Shift for reverse) to slew claw joints
  - Colour sensor (c) — request floor colour reading
  - Camera capture (p) — 10 images total (spec limit)
  - E-Stop (e) — can be triggered from either laptop


Usage:
  python3 pi_claw.py
  python3 pi_claw.py <pi_hostname_or_ip>   # if not localhost
"""

import struct, time, sys, select, tty, termios
import threading
import socket as sock_mod
import alex_camera


# ----------------------------------------------------------------
# SOCKET CLIENT
# ----------------------------------------------------------------


SOCKET_HOST = sys.argv[1] if len(sys.argv) > 1 else 'localhost'
SOCKET_PORT = 9000
_sock = None


def connectSocket():
    global _sock
    _sock = sock_mod.socket(sock_mod.AF_INET, sock_mod.SOCK_STREAM)
    print(f"Connecting to movement operator at {SOCKET_HOST}:{SOCKET_PORT}...")
    _sock.connect((SOCKET_HOST, SOCKET_PORT))
    print("Connected.\n")


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
COMMAND_CLAW_BASE     = 8  #unused
COMMAND_CLAW_SHOULDER = 9  #unused
COMMAND_CLAW_ELBOW    = 10 #unused
COMMAND_CLAW_GRIPPER  = 11 #unused
COMMAND_CLAW_HOME     = 12
COMMAND_CLAW_SPEED    = 13 #unused
COMMAND_OPEN_GRIPPER = 14
COMMAND_CLOSE_GRIPPER = 15
COMMAND_STOP_CLAW = 16
COMMAND_INCR_CLAW_BASE = 17
COMMAND_DECR_CLAW_BASE = 18
COMMAND_INCR_CLAW_SHOULDER = 19
COMMAND_DECR_CLAW_SHOULDER = 20
COMMAND_INCR_CLAW_ELBOW = 21
COMMAND_DECR_CLAW_ELBOW = 22
COMMAND_INCR_CLAW_SPEED = 23
COMMAND_DECR_CLAW_SPEED = 24
COMMAND_CLAW_P1 = 25
COMMAND_CLAW_P2 = 26
COMMAND_CLAW_P3 = 27
COMMAND_CLAW_P4 = 28
COMMAND_CLAW_P5 = 29
COMMAND_CLAW_P6 = 30
COMMAND_STOP_COLOUR = 31

RESP_OK          = 0
RESP_STATUS      = 1
RESP_COLOUR_DATA = 2
RESP_DEBUG_MESSAGE = 3

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

def sendCommand(cmdType, data=b'', params=None):
    """Send a framed command through the socket to pi_movement.py."""
    frame = packFrame(PACKET_TYPE_COMMAND, cmdType, data=data, params=params)
    _sock.sendall(frame)

# ----------------------------------------------------------------
# E-STOP STATE  (kept in sync via relayed RESP_STATUS packets)
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING

# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def _print(msg):
    """Print safely from any thread while in raw tty mode."""
    sys.stdout.write('\r' + msg + '\r\n')
    sys.stdout.flush()

def printPacket(pkt):
    global _estop_state
    ptype, cmd = pkt['packetType'], pkt['command']
    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            _print("Response: OK")

        elif cmd == RESP_STATUS:
            _estop_state = pkt['params'][0]
            if _estop_state == STATE_RUNNING: _print("Status: RUNNING")
            else:
                #todo HALT LIDAR
                _print("Status:STOPPED")

        elif cmd == RESP_COLOUR_DATA:
            r, g, b = pkt['params'][0], pkt['params'][1], pkt['params'][2]
            _print(f"Color  R:{r}  G:{g}  B:{b}  Hz")
        elif cmd == RESP_DEBUG_MESSAGE:
            msg = pkt['params']
            _print(f"msg:{msg[0]}, {msg[1]}, {msg[2]}")


        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            _print(f"Arduino: {debug}")
    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        _print(f"Arduino: {msg}")

# ----------------------------------------------------------------
# SOCKET RECEIVE THREAD
# Runs in background, prints relayed Arduino responses as they arrive
# ----------------------------------------------------------------

def _recv_thread():
    buf = b''
    while True:
        try:
            data = _sock.recv(256)
            if not data:
                _print("[Socket] Disconnected from movement operator.")
                break
            buf += data
            while len(buf) >= FRAME_SIZE:
                frame = buf[:FRAME_SIZE]
                buf   = buf[FRAME_SIZE:]
                if frame[:2] == MAGIC:
                    raw = frame[2: 2 + TPACKET_SIZE]
                    cs  = frame[-1]
                    if computeChecksum(raw) == cs:
                        printPacket(unpackTPacket(raw))
                    else:
                        _print("[Socket] Checksum mismatch — frame dropped.")
                else:
                    _print("[Socket] Bad magic bytes — resyncing.")
                    buf = b''
        except Exception as e:
            _print(f"[Socket] Recv error: {e}")
            break
    _print("[Socket] Receive thread exited.")

# ----------------------------------------------------------------
# CAMERA
# ----------------------------------------------------------------

MAX_FRAMES = 15
_camera          = alex_camera.cameraOpen()
_frames_remaining = MAX_FRAMES #10   # spec: 10 images total

def handleCameraCommand():
    global _frames_remaining
    if _estop_state == STATE_STOPPED:
        _print("Refused: E-Stop active.")
        return
    if _frames_remaining <= 0:
        _print(f"No frames remaining (limit: {MAX_FRAMES}).")
        return
    frame = alex_camera.captureGreyscaleFrame(_camera)
    alex_camera.renderGreyscaleFrame(frame)
    _frames_remaining -= 1
    _print(f"Frames remaining: {_frames_remaining}")


# ----------------------------------------------------------------
# CLAW — hold to slew
# ----------------------------------------------------------------
NEW_CLAW_KEYS = {
    'j': COMMAND_INCR_CLAW_BASE, #base left
    'l': COMMAND_DECR_CLAW_BASE, #base right
    'i': COMMAND_INCR_CLAW_SHOULDER, #shoulder up
    'k': COMMAND_DECR_CLAW_SHOULDER, #shoulder down
    'u': COMMAND_INCR_CLAW_ELBOW, #elbow out
    'n': COMMAND_DECR_CLAW_ELBOW, #elbow in
}

def _new_slew_claw(key):
    # params = [NEW_CLAW_KEYS[key]] + [0] * (PARAMS_COUNT - 1)
    sendCommand(NEW_CLAW_KEYS[key])


# ----------------------------------------------------------------
# SINGLE-TAP HANDLER  (non-claw keys)
# ----------------------------------------------------------------

def _handle_tap(ch):

    if ch == 'e':
        sendCommand(COMMAND_ESTOP, data=b'estop')
        _print("E-Stop toggled.")

    elif _estop_state == STATE_STOPPED: _print("Refused: E-Stop active.")
    else:

        if ch == 'c':
            sendCommand(COMMAND_GET_COLOUR)
            _print("Colour sensor ON")
        elif ch == 'x':
            sendCommand(COMMAND_STOP_COLOUR)
            _print("Colour sensor OFF")

        elif ch == 'p': handleCameraCommand()

        elif ch == 'h':
            sendCommand(COMMAND_CLAW_HOME)
            _print("Claw homing...")
        elif ch == '1':
            sendCommand(COMMAND_CLAW_P1)
            _print("Claw going to P1...")
        elif ch == '2':
            sendCommand(COMMAND_CLAW_P2)
            _print("Claw going to P2...")
        elif ch == '3':
            sendCommand(COMMAND_CLAW_P3)
            _print("Claw going to P3...")
        elif ch == '4':
            sendCommand(COMMAND_CLAW_P4)
            _print("Claw going to P4...")
        elif ch == '5':
            sendCommand(COMMAND_CLAW_P5)
            _print("Claw going to P5...")
        elif ch == '6':
            sendCommand(COMMAND_CLAW_P6)
            _print("Claw going to P6...")
        elif ch == '[':
            sendCommand(COMMAND_OPEN_GRIPPER)
            _print("opening gripper...")
        elif ch == ']':
            sendCommand(COMMAND_CLOSE_GRIPPER)
            _print("closing gripper...")
        elif ch == '=': #this is the '+' key without holding shift 
            sendCommand(COMMAND_INCR_CLAW_SPEED)
            _print("increasing claw speed")
        elif ch == '-':
            sendCommand(COMMAND_DECR_CLAW_SPEED)
            _print("decreasing claw speed")
        else:
            _print(f"Unknown key '{ch}'. Valid: e cx p h123456 ij kl un [ ] +-  q")


# ----------------------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------------------


def runCommandInterface():
    print("=== Laptop 2 — Claw operator ===")
    print(f"Frames remaining: {_frames_remaining}")
    print("Hold i/o/k/n: slew joint fwd   Shift+same: slew bwd")
    print("c: colour   p: camera   h: home claw   v: claw speed   e: E-Stop   q: quit\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_slew_key = None

    try:
        tty.setraw(fd)


        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.08)

            if not r:
                if last_slew_key is not None:
                    sendCommand(COMMAND_STOP_CLAW)
                    #_print("stopping claw 1")
                    last_slew_key = None
                continue

            ch      = sys.stdin.read(1)
            ch_lower = ch.lower()

            if ch_lower in ('q', '\x03'):  # q or Ctrl+C
                break
              
            elif ch_lower in NEW_CLAW_KEYS:
                if _estop_state == STATE_STOPPED:
                    print("\r Refused: E-Stop active.")
                    continue
                if ch_lower != last_slew_key:
                    last_slew_key = ch_lower
                    _new_slew_claw(ch_lower)
                  
            else:
                if last_slew_key is not None:
                    sendCommand(COMMAND_STOP_CLAW)
                    last_slew_key = None

                _handle_tap(ch_lower)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        #stop/reset everything
        sendCommand(COMMAND_STOP_MOTORS)
        sendCommand(COMMAND_CLAW_HOME)
        sendCommand(COMMAND_STOP_COLOUR)
        sys.stdout.write("\r\nClaw operator exiting.\r\n")
        sys.stdout.flush()

# ----------------------------------------------------------------
# ENTRY POINT
# ----------------------------------------------------------------


if __name__ == '__main__':
    connectSocket()


    # Background thread receives and prints Arduino responses relayed from pi_movement
    t = threading.Thread(target=_recv_thread, daemon=True)
    t.start()


    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        _sock.close()
        alex_camera.cameraClose(_camera)



