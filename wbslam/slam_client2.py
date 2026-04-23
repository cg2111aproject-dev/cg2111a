#!/usr/bin/env python3
import json
import math
import socket
import sys
import time
from collections import deque

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow
import numpy as np

# --- Connection settings ---
PI_HOST = '100.74.100.73'
PI_PORT = 5002

# --- SLAM settings ---
MAP_SIZE_PIXELS  = 800
MAP_SIZE_METERS  = 6
MAP_QUALITY      = 2
HOLE_WIDTH_MM    = 200
SCAN_SIZE        = 360
SCAN_RATE_HZ     = 5
DETECTION_ANGLE  = 360
MAX_DISTANCE_MM  = 12000

# Robot size (with safety buffer)
ROBOT_WIDTH  = 0.18   # 15cm + 3cm padding
ROBOT_LENGTH = 0.35   # 25cm + 3cm padding
LIDAR_X_OFFSET = 0.01 # 2cm from the rear edge

# Point cloud display range (metres from robot centre)
PC_RANGE_M = 3.0

SCAN_ANGLES = [float(i) for i in range(SCAN_SIZE)]

beam_history = [deque(maxlen=5) for _ in range(SCAN_SIZE)]


def smooth_distances(distances):
    out = []
    for i, d in enumerate(distances):
        if 0 < d < MAX_DISTANCE_MM:
            beam_history[i].append(d)
        if beam_history[i]:
            out.append(int(np.median(beam_history[i])))
        else:
            out.append(0)
    return out


def make_slam():
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser
    laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM)
    slam  = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS,
                      hole_width_mm=HOLE_WIDTH_MM, map_quality=MAP_QUALITY)
    return slam


class SocketReader:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = b''

    def read_line(self):
        try:
            while b'\n' not in self.buffer:
                chunk = self.sock.recv(8192)
                if not chunk:
                    return None
                self.buffer += chunk
        except BlockingIOError:
            pass
        if b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            return line.decode('utf-8')
        return None


def polar_to_xy_robot_frame(distances, angles_deg):
    """
    Convert LIDAR scan to (x, y) points in the robot's local frame.
    Robot faces +x. Returns arrays of (x, y) for valid readings only.
    """
    xs, ys = [], []
    for dist_mm, angle_deg in zip(distances, angles_deg):
        if dist_mm <= 0 or dist_mm >= 6000:
            continue
        dist_m = dist_mm / 1000.0
        rad = math.radians(angle_deg)
        xs.append(-dist_m * math.sin(rad))
        ys.append(dist_m * math.cos(rad))
    return np.array(xs), np.array(ys)


def run():
    for backend in ['TkAgg', 'Qt5Agg', 'MacOSX', 'Agg']:
        try:
            matplotlib.use(backend)
            break
        except Exception:
            continue

    slam     = make_slam()
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    persistent_map = np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8)

    # --- Two-panel figure: point cloud left, SLAM map right ---
    fig, (ax_pc, ax_map) = plt.subplots(1, 2, figsize=(18, 9))
    fig.patch.set_facecolor('#FFFFFF')

    path_color  = '#2196F3'
    robot_color = '#FF3333'

    # ── SLAM map panel (right) ──────────────────────────────────────────────
    ax_map.set_facecolor('#FFFFFF')
    ax_map.tick_params(colors='#000000')
    for spine in ax_map.spines.values():
        spine.set_edgecolor('#010101')

    map_img = ax_map.imshow(
        np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8),
        cmap='gray', vmin=0, vmax=255, origin='lower',
        extent=[0, MAP_SIZE_METERS, 0, MAP_SIZE_METERS],
        interpolation='nearest'
    )

    robot_dot_map, = ax_map.plot([], [], 'o', color=robot_color, markersize=8,
                                  label='Robot', zorder=10)
    robot_rect_map = Rectangle((0, 0), ROBOT_WIDTH, ROBOT_LENGTH,
                                linewidth=2, edgecolor=robot_color,
                                facecolor='none', zorder=5)
    ax_map.add_patch(robot_rect_map)

    path_x, path_y = [], []
    path_line, = ax_map.plot([], [], '-', color=path_color,
                              linewidth=3, alpha=0.8, zorder=4)

    heading_arrow_map = None
    ax_map.set_title('SLAM Map', color='#000000')
    # ── Point cloud panel (left) ────────────────────────────────────────────
    ax_pc.set_facecolor('#FFFFFF')
    ax_pc.set_xlim(-PC_RANGE_M, PC_RANGE_M)
    ax_pc.set_ylim(-PC_RANGE_M, PC_RANGE_M)
    ax_pc.set_aspect('equal')
    ax_pc.tick_params(colors='#AAAAAA')
    ax_pc.set_title('Point Cloud (robot frame)', color='#DDDDDD')
    ax_pc.set_xlabel('x (m)', color='#AAAAAA')
    ax_pc.set_ylabel('y (m)', color='#AAAAAA')
    for spine in ax_pc.spines.values():
        spine.set_edgecolor('#444444')

    # Range rings
    for r in [1.0, 2.0, 3.0]:
        circle = plt.Circle((0, 0), r, color='#333333', fill=False,
                             linewidth=0.8, linestyle='--', zorder=1)
        ax_pc.add_patch(circle)
        ax_pc.text(r * 0.707 + 0.05, r * 0.707 + 0.05, f'{r:.0f}m',
                   color='#555555', fontsize=7, zorder=2)

    # Crosshairs at robot centre
    ax_pc.axhline(0, color='#333333', linewidth=0.5, zorder=1)
    ax_pc.axvline(0, color='#333333', linewidth=0.5, zorder=1)

    # Point cloud scatter
    pc_scatter = ax_pc.scatter([], [], s=8, c="#111111", alpha=0.45, zorder=5)

    # Robot rectangle on point cloud (robot is always at centre, facing +x)
    robot_rect_pc = Rectangle(
        (-ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2),
        ROBOT_WIDTH, ROBOT_LENGTH,
        linewidth=2, edgecolor=robot_color, facecolor='none', zorder=6
    )
    ax_pc.add_patch(robot_rect_pc)

    # Robot dot at centre of point cloud
    robot_dot_pc, = ax_pc.plot([0], [0], 'o', color=robot_color,
                                markersize=8, zorder=7)

    # Heading arrow on point cloud (always points +x = forward)
    heading_arrow_pc = ax_pc.annotate(
        '', xy=(0, ROBOT_LENGTH / 2), xytext=(0, -ROBOT_LENGTH / 2),
        arrowprops=dict(arrowstyle='fancy', color='#0D47A1',
                        mutation_scale=20, lw=0, fc='#0D47A1'),
        zorder=8
    )
    

    plt.tight_layout()
    plt.ion()
    plt.show()

    print(f'[slam_client] Connecting to {PI_HOST}:{PI_PORT} ...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((PI_HOST, PI_PORT))
        sock.setblocking(False)
        reader = SocketReader(sock)
        print('[slam_client] Connected!')
    except Exception:
        print("Could not connect to Pi.")
        return

    prev_distances = None
    frame_count = 0
    heading_arrow_map = None

    try:
        while plt.fignum_exists(fig.number):
            # Drain to latest frame only
            latest_raw = None
            while True:
                raw = reader.read_line()
                if not raw:
                    break
                latest_raw = raw

            if not latest_raw:
                plt.pause(0.01)
                continue

            try:
                data = json.loads(latest_raw)
                raw_distances = data['distances']
                display_distances = smooth_distances(raw_distances)
                frame_count += 1

                # ── SLAM update ──────────────────────────────────────────
                masked = raw_distances
                valid = sum(1 for d in masked if 0 < d < MAX_DISTANCE_MM)
                if valid > 30:
                    slam.update(masked, scan_angles_degrees=SCAN_ANGLES)
                    prev_distances = masked
                elif prev_distances:
                    slam.update(prev_distances, scan_angles_degrees=SCAN_ANGLES)

                x_mm, y_mm, theta_deg = slam.getpos()
                slam.getmap(mapbytes)

                rx, ry = x_mm / 1000.0, y_mm / 1000.0
                rad = math.radians(theta_deg)

                # ── SLAM map panel update ────────────────────────────────
                angle_rad = math.radians(theta_deg + 90)
                cx = rx - (ROBOT_WIDTH / 2 * math.cos(angle_rad) - ROBOT_LENGTH / 2 * math.sin(angle_rad))
                cy = ry - (ROBOT_WIDTH / 2 * math.sin(angle_rad) + ROBOT_LENGTH / 2 * math.cos(angle_rad))

                robot_rect_map.set_xy((cx, cy))
                robot_rect_map.angle = theta_deg + 90
                robot_dot_map.set_data([rx], [ry])
                path_x.append(rx)
                path_y.append(ry)
                if len(path_x) > int(1e9):
                    path_x.pop(0)
                    path_y.pop(0)
                path_line.set_data(path_x, path_y)

                if heading_arrow_map:
                    heading_arrow_map.remove()
                dx, dy = -0.4 * math.cos(rad), -0.4 * math.sin(rad)
                base_x = rx + (ROBOT_LENGTH / 2) * math.cos(rad)
                base_y = ry + (ROBOT_LENGTH / 2) * math.sin(rad)
                heading_arrow_map = ax_map.annotate(
                    '', xy=(base_x + dx, base_y + dy),
                    xytext=(base_x, base_y),
                    arrowprops=dict(arrowstyle='fancy', color='#0D47A1',
                                    mutation_scale=20, lw=0, fc='#0D47A1')
                )

                arr = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
                    MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)

                rise_mask = arr > persistent_map
                free_mask = arr > 160
                persistent_map[rise_mask & free_mask]  = arr[rise_mask & free_mask]
                persistent_map[rise_mask & ~free_mask] = arr[rise_mask & ~free_mask]
                blend_free = 0.40 if frame_count < 75 else 0.08
                persistent_map[~rise_mask & free_mask] = (
                    persistent_map[~rise_mask & free_mask] * (1 - blend_free)
                    + arr[~rise_mask & free_mask] * blend_free
                ).astype(np.uint8)
                blend_occ = 0.25 if frame_count < 75 else 0.005
                persistent_map[~rise_mask & ~free_mask] = (
                    persistent_map[~rise_mask & ~free_mask] * (1 - blend_occ)
                    + arr[~rise_mask & ~free_mask] * blend_occ
                ).astype(np.uint8)

                arr_float    = persistent_map.astype(np.float32)
                arr_stretched = np.clip(
                    (arr_float - 160) / (240 - 160) * 255, 0, 255
                ).astype(np.uint8)
                map_img.set_data(arr_stretched)
                ax_map.set_title(
                    f'SLAM Map  x={x_mm:.0f}mm  y={y_mm:.0f}mm  θ={theta_deg:.1f}°',
                    color='#000000'
                )

                # ── Point cloud panel update ─────────────────────────────
                # Convert all 360 readings to robot-frame Cartesian coords.
                # The robot is always at (0, 0) facing +x in this frame.
                pc_x, pc_y = polar_to_xy_robot_frame(display_distances, SCAN_ANGLES)
                if len(pc_x) > 0:
                    pc_scatter.set_offsets(np.column_stack([pc_x, pc_y]))
                else:
                    pc_scatter.set_offsets(np.empty((0, 2)))

                ax_pc.set_title(
                    f'Point Cloud (robot frame)  {len(pc_x)} pts',
                    color='#DDDDDD'
                )

            except Exception:
                continue

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.001)

    except KeyboardInterrupt:
        print('\n[slam_client] Exiting.')
    finally:
        sock.close()
        plt.close()


if __name__ == '__main__':
    run()
