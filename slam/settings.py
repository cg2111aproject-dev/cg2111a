#!/usr/bin/env python3
"""
settings.py - All user-configurable settings for the SLAM system.

Change the values in this file to tune the SLAM system for your robot.
The most common settings to change are LIDAR_PORT and LIDAR_OFFSET_DEG.
"""

# ===========================================================================
# LIDAR hardware settings
# ===========================================================================

# Serial port the RPLidar is plugged into.
# On the Raspberry Pi with a USB adapter this is usually /dev/ttyUSB0.
# If you have other USB devices connected it might be /dev/ttyUSB1, etc.
LIDAR_PORT = '/dev/ttyUSB0'

# Baud rate for the RPLidar A1M8. Do not change this.
LIDAR_BAUD = 115200

# ===========================================================================
# SLAM map settings
# ===========================================================================

# Side length of the square occupancy map in pixels.
MAP_SIZE_PIXELS = 1000

# Real-world area the map covers, in metres.
MAP_SIZE_METERS = 8

# How aggressively new LIDAR scans update the map (1 = slow, 10 = fast).
MAP_QUALITY = 5

# Maximum gap (in mm) that BreezySLAM treats as a continuous wall.
HOLE_WIDTH_MM = 100

# ===========================================================================
# Scan settings
# ===========================================================================

SCAN_SIZE = 360
SCAN_RATE_HZ = 5
DETECTION_ANGLE = 360
MAX_DISTANCE_MM = 12000

# ===========================================================================
# LIDAR mounting offset
# ===========================================================================

# Rotate all LIDAR readings by this many degrees before feeding them to SLAM.
# 0 = LIDAR forward matches robot forward.
LIDAR_OFFSET_DEG = 0

# ===========================================================================
# Scan quality thresholds
# ===========================================================================

MIN_VALID_POINTS = 150
INITIAL_ROUNDS_SKIP = 5

# ===========================================================================
# UI and rendering settings
# ===========================================================================

# How many times per second the terminal map refreshes.
# Increased from 2 to 4 for smoother display.
UI_REFRESH_HZ = 4

# Maximum width and height of the rendered map in terminal cells.
MAX_RENDER_COLS = 120
MAX_RENDER_ROWS = 45

# How often the map is copied from the SLAM process.
# Increased from 1 Hz to 4 Hz for smoother map updates.
# The actual copy is skipped when map content is unchanged (hash check),
# so increasing this has low CPU cost in practice.
MAP_UPDATE_HZ = 4.0
MAP_UPDATE_INTERVAL = 1.0 / MAP_UPDATE_HZ

# Default zoom level index into ZOOM_HALF_M (0 = full map).
DEFAULT_ZOOM = 0

# How far to pan per key-press, as a fraction of the current view half-width.
PAN_STEP_FRACTION = 0.20

# Byte value written to uninitialised map cells.
# BreezySLAM uses 0 = wall, 127 = unknown, 255 = free.
UNKNOWN_BYTE = 127

# Zoom levels: None means show the full map.
ZOOM_HALF_M = [
    None,
    MAP_SIZE_METERS / 2.0,
    MAP_SIZE_METERS / 3.0,
    MAP_SIZE_METERS / 5.0,
    MAP_SIZE_METERS / 8.0,
]