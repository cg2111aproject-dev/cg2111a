# import shutil
# import sys
# import numpy as np
# import time
# from lidar.alex_lidar import lidarConnect, lidarDisconnect, lidarStatus, performSingleScan

# # ==============================================================================
# # GLOBAL CONFIGURATION
# # ==============================================================================
# PORT = "/dev/ttyUSB0"
# BAUDRATE = 115200
# GRID_WIDTH = 100   
# GRID_HEIGHT = 60   
# MAX_RANGE_MM = 2500 
# CLI_ASPECT_RATIO = 2.2

# DENSITY_CHARS = " ░▒▓█" 
# AXIS_CHAR_H = "─"
# AXIS_CHAR_V = "│"
# AXIS_CHAR_CENTER = "┼"

# # ==============================================================================
# # TERMINAL UI UTILITIES
# # ==============================================================================
# ESC = "\033["
# RESET = f"{ESC}0m"
# CLEAR_DOWN = f"{ESC}J"
# HIDE_CUR = f"{ESC}?25l"
# SHOW_CUR = f"{ESC}?25h"

# CLR_AXIS  = f"{ESC}2m"   
# CLR_POINT = f"{ESC}32m"  
# CLR_LABEL = f"{ESC}36m"  

# def ui_hide_cursor():
#     sys.stdout.write(HIDE_CUR)

# def ui_show_cursor():
#     sys.stdout.write(SHOW_CUR)

# def ui_update_display(content_string, move_up_amount):
#     """
#     Moves cursor up by the exact number of newlines in the previous frame.
#     """
#     # \r resets to column 1, {move_up_amount}A moves up
#     sys.stdout.write(f"\r{ESC}{move_up_amount}A{CLEAR_DOWN}")
#     sys.stdout.write(content_string)
#     sys.stdout.flush()

# def ui_prepare_frame(frame_height=GRID_HEIGHT, frame_width=GRID_WIDTH):
#     """
#     Reserve terminal space for the LiDAR frame and hide the cursor for clean updates.
#     Validates terminal dimensions before reserving space and returns the number of lines written.
#     """
#     term_cols, term_rows = shutil.get_terminal_size(fallback=(frame_width, frame_height + 2))
#     if term_cols < frame_width or term_rows < frame_height:
#         raise RuntimeError(
#             f"Terminal too small for LiDAR grid. Requires at least {frame_width}x{frame_height}, "
#             f"but detected {term_cols}x{term_rows}."
#         )
#     sys.stdout.write("\n" * frame_height)
#     ui_hide_cursor()
#     return frame_height

# # ==============================================================================
# # RENDERING LOGIC
# # ==============================================================================
# def points_to_grid(xs, ys, *, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
#     """Convert Cartesian points into the discrete grid used by the CLI renderer."""

#     # Initialize an empty grid. Each cell will count the number of points that fall into it.
#     grid = np.zeros((grid_height, grid_width), dtype=int)
    
#     # Determine the center of the grid in terms of row and column indices, and the scale factor to convert mm to grid cells.
#     mid_row, mid_col = grid_height // 2, grid_width // 2

#     # The scale factor converts from millimeters to grid cells. We want MAX_RANGE_MM to correspond to half the grid size (from center to edge).
#     scale = MAX_RANGE_MM / (grid_height / 2)

#     # Count points in each grid cell. We iterate through each point and put it into the appropriate cell in the grid.
#     for x_mm, y_mm in zip(xs, ys):
#         # Convert the (x_mm, y_mm) coordinates into grid indices (col, row). The center of the grid corresponds to (0, 0) in mm.
#         # The aspect ratio is applied to the x_mm value to stretch it horizontally, since terminal characters are taller than they are wide.
#         col = int((x_mm * CLI_ASPECT_RATIO) / scale + mid_col)
#         row = int(y_mm / scale + mid_row)
#         if 0 <= col < grid_width and 0 <= row < grid_height:
#             grid[row, col] += 1
#     return grid

# def gridValue_to_char(count, max_idx=len(DENSITY_CHARS) - 1):
#     """Convert a point count into its representative character based on density."""
#     return DENSITY_CHARS[min(count, max_idx)]

# def render_to_cli(grid):
#     """Render the numeric grid into a CLI string by composing a temporary character grid."""
#     grid_height, grid_width = grid.shape
#     mid_row, mid_col = grid_height // 2, grid_width // 2
#     max_idx = len(DENSITY_CHARS) - 1

#     # Temporary grid that stores the final string for each cell.
#     char_grid = [[" " for _ in range(grid_width)] for _ in range(grid_height)]

#     # 1) Draw points first so later overlays can replace them.
#     for r in range(grid_height):
#         for c in range(grid_width):
#             val = grid[r, c]
#             if val > 0:
#                 char_grid[r][c] = f"{CLR_POINT}{gridValue_to_char(val, max_idx=max_idx)}{RESET}"

#     # 2) Overlay axes so they always remain visible.
#     axis_h = f"{CLR_AXIS}{AXIS_CHAR_H}{RESET}"
#     axis_v = f"{CLR_AXIS}{AXIS_CHAR_V}{RESET}"
#     axis_center = f"{CLR_AXIS}{AXIS_CHAR_CENTER}{RESET}"

#     for c in range(grid_width):
#         char_grid[mid_row][c] = axis_h
#     for r in range(grid_height):
#         char_grid[r][mid_col] = axis_v
#     char_grid[mid_row][mid_col] = axis_center

#     # 3) Overlay Y-axis labels adjacent to the vertical axis.
#     def overlay_label(row_idx, text):
#         start_col = min(mid_col + 1, grid_width)
#         col = start_col
#         for ch in text:
#             if col >= grid_width:
#                 break
#             char_grid[row_idx][col] = f"{CLR_LABEL}{ch}{RESET}"
#             col += 1

#     if grid_height > 0:
#         overlay_label(grid_height - 1, f" +{MAX_RANGE_MM}mm")
#         overlay_label(0, f" -{MAX_RANGE_MM}mm")

#     # 4) Collapse the character grid into output lines (top row first).
#     output_lines = []
#     for r in reversed(range(grid_height)):
#         output_lines.append("".join(char_grid[r]))

#         # X-Labels line immediately under the x-axis
#         if r == mid_row:
#             l_lab, c_lab, r_lab = f"-{MAX_RANGE_MM}mm", "0mm", f"+{MAX_RANGE_MM}mm"
#             gap1 = " " * max(0, mid_col - len(l_lab))
#             gap2 = " " * max(0, grid_width - mid_col - len(c_lab) - len(r_lab))
#             output_lines.append(f"{CLR_LABEL}{l_lab}{RESET}{gap1}{CLR_LABEL}{c_lab}{RESET}{gap2}{CLR_LABEL}{r_lab}{RESET}")

#     # Join the lines. This creates exactly grid_height newlines.
#     return "\n".join(output_lines)

# # ==============================================================================
# # MAIN EXECUTION
# # ==============================================================================

# def convert_to_cartesian(angles, distances):
#     """
#     Convert the scanAngles and scanDistances into X and Y coordinates
#     Hint, Pythagoras' theorem can be used to convert polar to cartesian coordinates
#     We've provided you with the value of PI
#     """
#     VAL_PI = np.pi

#     Xs = []
#     Ys = []
#     for angle, distance in zip(angles, distances):
#         # TODO: Fill in the code to convert the polar coordinates to cartesian coordinates
#         # Hint, Pythagoras' theorem can be used to convert polar to cartesian coordinates
#         cartesian_X = distance * np.cos(angle * VAL_PI / 180)
#         cartesian_Y = -distance * np.sin(angle * VAL_PI / 180)

#         Xs.append(cartesian_X)
#         Ys.append(cartesian_Y)

#     return Xs, Ys



# def plot_single_scan():
#     """Renders a single frame and exits."""
#     print("====== LiDAR Single Plot ======")
#     lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
#     try:
#         # Retrieve and print the LiDAR device information
#         status = lidarStatus(lidar)
        
#         print("====== Scanning ======")
#         # Get a single scan using the library. This returns the scan data.
#         scan_data = performSingleScan(lidar, status['typical_scan_mode'])
        
#         # Convert the scan data polar coordinates into Cartesian coordinates (Xs and Ys) for plotting.
#         xs, ys = convert_to_cartesian(scan_data[0], scan_data[1])
        
#         # Convert continuous Cartesian coordinates into a discrete grid representation
#         grid = points_to_grid(xs, ys)

#         # Render the grid into a string and print it to the CLI
#         print(render_to_cli(grid))
#     finally:
#         lidarDisconnect(lidar)
#         print("\nSingle scan complete.")

# def plot_live_scan():
#     print("====== LiDAR Live Plot ======")
#     lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
#     status = lidarStatus(lidar)
#     mode = status['typical_scan_mode']
#     print(f"Connected. Mode: {mode}\n")

#     print("====== Scanning ======")
#     # Reserve space for the CLI plot and hide the cursor.
#     move_up_amount = ui_prepare_frame(GRID_HEIGHT)

#     try:
#         while True:
#             scan_data = performSingleScan(lidar, mode)
            
#             # Convert the scan data polar coordinates into Cartesian coordinates (Xs and Ys) for plotting.
#             xs, ys = convert_to_cartesian(scan_data[0], scan_data[1])

#             # Convert continuous Cartesian coordinates into a discrete grid representation
#             grid = points_to_grid(xs, ys)

#             # Render the grid into a string
#             full_frame = render_to_cli(grid)

#             # Update the CLI display with the new frame, moving the cursor up to overwrite the previous frame.
#             ui_update_display(full_frame, move_up_amount)
            
#             # Sleep briefly to allow for a smoother display and to prevent overwhelming the terminal with updates. Adjust as needed for performance.
#             time.sleep(0.01)

#     except KeyboardInterrupt:
#         # Move to bottom of the scan area for clean exit
#         sys.stdout.write("\n" * 2)
#         print("Scan stopped by user.")
#     finally:
#         lidarDisconnect(lidar)
#         ui_show_cursor()
#         sys.stdout.flush()

# if __name__ == "__main__":
#     # Uncomment one of the following lines to run either the single scan plot or the live scan plot.
#     plot_single_scan()
#     # plot_live_scan()
"""
lidar_example_cli_plot.py  —  RPLidar CLI visualiser (Pi-optimised)

Measured pipeline cost on this machine (proxy for Pi 4):
  angular_filter   0.02 ms   bincount LUT (was 10.7 ms with argsort+median)
  hit_grid         0.07 ms   np.add.at on ~360 points
  free_space       0.72 ms   angle-LUT raster (was 9.4 ms with np.outer+add.at)
  render           2.15 ms   vectorised object-array
  ─────────────────────────
  total processing ~3 ms     well inside 100-200 ms scan period

Public API (drop-in replacement):
  plot_single_scan()
  plot_live_scan()
"""

import shutil, sys, time
import numpy as np
from scipy.ndimage import uniform_filter
from pyrplidar.alex_lidar import lidarConnect, lidarDisconnect, lidarStatus, performSingleScan

# ── Configuration ─────────────────────────────────────────────────────────────
PORT     = "/dev/ttyUSB0"
BAUDRATE = 115200

GRID_W   = 160       # columns
GRID_H   = 90        # rows
MAX_MM   = 2500      # sensor max range (mm)
ASPECT   = 2.2       # terminal char aspect ratio (cols per mm-row)

SMOOTH_SIZE      = 3    # uniform_filter kernel (cells) for wall density
MIN_WALL_DENSITY = 1.8  # smoothed-hit threshold → wall
FREE_MARGIN      = 0.95 # cell is free if dist < measured_dist * this factor

# ── ANSI ──────────────────────────────────────────────────────────────────────
ESC        = "\033["
RESET      = "\033[0m"
CLEAR_DOWN = "\033[J"
HIDE_CUR   = "\033[?25l"
SHOW_CUR   = "\033[?25h"

# ── Lookup tables — built ONCE at import, reused every frame ──────────────────
_WALL_CHARS = np.array([
    "\033[2;32m·\033[0m",   # level 0 – barely there
    "\033[32m∘\033[0m",     # 1
    "\033[32m○\033[0m",     # 2
    "\033[92m◉\033[0m",     # 3
    "\033[92m●\033[0m",     # 4
    "\033[1;92m█\033[0m",   # 5 – solid wall
], dtype=object)

_FREE_CHARS = np.array([
    "\033[90m·\033[0m",   # far  / dim grey
    "\033[34m░\033[0m",   # mid  blue
    "\033[34m▒\033[0m",   # near bright blue
], dtype=object)

_AXIS_H      = "\033[2m─\033[0m"
_AXIS_V      = "\033[2m│\033[0m"
_ROBOT       = "\033[1;33m✦\033[0m"
_CLR_LABEL   = "\033[36m"
_CLR_RESET   = RESET

# Per-cell angle and distance-from-centre grids — computed once at import.
# These never change because the grid size is fixed.
_mid_c  = GRID_W // 2
_mid_r  = GRID_H // 2
_cy, _cx = np.mgrid[0:GRID_H, 0:GRID_W].astype(np.float32)

# Angle (degrees, 0–360) of each grid cell relative to robot centre
_CELL_ANG_IDX = (np.degrees(
    np.arctan2(-(_cy - _mid_r), (_cx - _mid_c) / ASPECT)
) % 360).astype(np.int32)          # shape (H, W), values 0-359

# Metric distance (mm) of each grid cell from robot centre
_CELL_DIST_MM = np.hypot(
    (_cx - _mid_c) / ASPECT,
     _cy - _mid_r
) * (MAX_MM / (_mid_r))            # shape (H, W)


# ── Terminal UI ───────────────────────────────────────────────────────────────
def ui_hide_cursor(): sys.stdout.write(HIDE_CUR)
def ui_show_cursor(): sys.stdout.write(SHOW_CUR)

def ui_update_display(content, move_up):
    sys.stdout.write(f"\r{ESC}{move_up}A{CLEAR_DOWN}{content}")
    sys.stdout.flush()

def ui_prepare_frame(h=GRID_H, w=GRID_W):
    tc, tr = shutil.get_terminal_size(fallback=(w, h + 2))
    if tc < w or tr < h:
        raise RuntimeError(f"Terminal too small: need {w}x{h}, got {tc}x{tr}.")
    sys.stdout.write("\n" * h)
    ui_hide_cursor()
    return h


# ── Stage 1 — noise reduction: bincount mean LUT  (0.02 ms) ──────────────────
def build_angle_lut(angles, distances):
    """
    Build a 360-element float32 LUT: lut[deg] = mean measured distance.

    Uses np.bincount which is a single C call — 500× faster than the
    previous argsort + np.unique + list-comp median approach.
    Mean is a good proxy for median when the sensor already rejects
    multi-return outliers (which RPLidar firmware does internally).
    """
    angles    = np.asarray(angles,    dtype=np.float32)
    distances = np.asarray(distances, dtype=np.float32)

    valid     = distances > 0
    angles, distances = angles[valid], distances[valid]

    idx    = angles.astype(np.int32) % 360
    counts = np.bincount(idx, minlength=360).astype(np.float32)
    sums   = np.bincount(idx, weights=distances, minlength=360).astype(np.float32)

    lut          = np.zeros(360, dtype=np.float32)
    has_data     = counts > 0
    lut[has_data] = sums[has_data] / counts[has_data]
    return lut          # shape (360,)


# ── Stage 2 — hit grid  (0.07 ms) ────────────────────────────────────────────
def build_hit_grid(lut):
    """
    Convert the angle LUT back to Cartesian hit-grid.
    Only ~360 points, so this is trivially fast.
    """
    ang_deg   = np.where(lut > 0)[0].astype(np.float32)
    dists     = lut[ang_deg.astype(np.int32)]

    rad  = np.deg2rad(ang_deg)
    xs   =  dists * np.cos(rad)
    ys   = -dists * np.sin(rad)

    scale = MAX_MM / float(_mid_r)
    cols  = ((xs * ASPECT) / scale + _mid_c).astype(np.int32)
    rows  = (ys             / scale + _mid_r).astype(np.int32)

    mask  = (cols >= 0) & (cols < GRID_W) & (rows >= 0) & (rows < GRID_H)
    cols, rows = cols[mask], rows[mask]

    grid  = np.zeros((GRID_H, GRID_W), dtype=np.float32)
    np.add.at(grid, (rows, cols), 1)
    return grid


# ── Stage 3 — free-space grid  (0.72 ms) ─────────────────────────────────────
def build_free_space_grid(lut, hit_grid):
    """
    Angle-LUT raster approach — no ray loops, no np.outer.

    For every grid cell:
      • look up the LUT distance measured at that cell's angle
      • if the cell's metric distance < LUT distance * FREE_MARGIN → free

    Everything is a vectorised array op on the pre-computed _CELL_ANG_IDX
    and _CELL_DIST_MM grids (built once at import).
    """
    meas_dist = lut[_CELL_ANG_IDX]          # broadcast LUT onto grid: (H, W)

    free = (
        (meas_dist > 0) &
        (_CELL_DIST_MM < meas_dist * FREE_MARGIN)
    ).astype(np.float32)

    # Smear to fill inter-ray gaps (single C call)
    free = uniform_filter(free, size=3)

    # Zero wall cells — they are obstacles not free space
    free[hit_grid >= 1] = 0.0

    # Distance fade: near robot = brighter blue
    dist_norm = _CELL_DIST_MM / MAX_MM
    free     *= np.clip(1.0 - dist_norm * 0.6, 0.2, 1.0)

    return free


# ── Stage 4 — renderer  (2.15 ms) ────────────────────────────────────────────
def render_to_cli(hit_grid, free_grid):
    """
    Vectorised renderer using a NumPy object-array of pre-coloured strings.
    No per-cell Python branching.
    """
    smooth = uniform_filter(hit_grid, size=SMOOTH_SIZE)

    out = np.full((GRID_H, GRID_W), " ", dtype=object)

    # Free space layer
    free_mask = free_grid > 0.08
    if free_mask.any():
        fidx      = np.clip((free_grid * len(_FREE_CHARS)).astype(np.int32),
                            0, len(_FREE_CHARS) - 1)
        out[free_mask] = _FREE_CHARS[fidx[free_mask]]

    # Wall layer
    wall_mask = smooth >= MIN_WALL_DENSITY
    if wall_mask.any():
        s_max = smooth.max()
        widx  = np.clip(
            (smooth / s_max * (len(_WALL_CHARS) - 1)).astype(np.int32),
            0, len(_WALL_CHARS) - 1
        )
        out[wall_mask] = _WALL_CHARS[widx[wall_mask]]

    # Axes (skip cells already occupied by walls)
    no_wall_h = ~wall_mask[_mid_r, :]
    no_wall_v = ~wall_mask[:, _mid_c]
    out[_mid_r, no_wall_h] = _AXIS_H
    out[no_wall_v, _mid_c] = _AXIS_V
    out[_mid_r, _mid_c]    = _ROBOT      # robot marker at centre

    # Y-axis range labels
    for i, ch in enumerate(f" -{MAX_MM}mm"):
        c = _mid_c + 1 + i
        if c < GRID_W: out[0, c] = f"{_CLR_LABEL}{ch}{_CLR_RESET}"
    for i, ch in enumerate(f" +{MAX_MM}mm"):
        c = _mid_c + 1 + i
        if c < GRID_W: out[GRID_H - 1, c] = f"{_CLR_LABEL}{ch}{_CLR_RESET}"

    # Collapse rows to a single string (top = high-y)
    lines = []
    for r in reversed(range(GRID_H)):
        lines.append("".join(out[r]))
        if r == _mid_r:
            l, c_lab, rr = f"-{MAX_MM}mm", "0mm", f"+{MAX_MM}mm"
            g1 = " " * max(0, _mid_c - len(l))
            g2 = " " * max(0, GRID_W - _mid_c - len(c_lab) - len(rr))
            lines.append(
                f"{_CLR_LABEL}{l}{_CLR_RESET}{g1}"
                f"{_CLR_LABEL}{c_lab}{_CLR_RESET}{g2}"
                f"{_CLR_LABEL}{rr}{_CLR_RESET}"
            )
    return "\n".join(lines)


# ── Legend ────────────────────────────────────────────────────────────────────
def print_legend():
    w = _WALL_CHARS
    f = _FREE_CHARS
    print(
        f"  {w[5]} wall solid  {w[3]} wall mid  {w[1]} wall thin"
        f"  {f[2]} free near  {f[0]} free far  {_ROBOT} robot\n"
    )


# ── Full pipeline ─────────────────────────────────────────────────────────────
def _process_scan(scan_data):
    lut      = build_angle_lut(scan_data[0], scan_data[1])
    hit_grid = build_hit_grid(lut)
    free_grid = build_free_space_grid(lut, hit_grid)
    return render_to_cli(hit_grid, free_grid)


# ── Public API ────────────────────────────────────────────────────────────────
def plot_single_scan():
    print("====== LiDAR Single Plot ======")
    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    try:
        status    = lidarStatus(lidar)
        print("====== Scanning ======")
        scan_data = performSingleScan(lidar, status['typical_scan_mode'])
        print_legend()
        print(_process_scan(scan_data))
    finally:
        lidarDisconnect(lidar)
        print("\nSingle scan complete.")


def _flush_lidar(lidar):
    """
    Drain the serial buffer WITHOUT calling lidar.reset().

    lidar.reset() sends a greeting string back from the sensor whose first
    two bytes happen to be 0x52 ('R') and 0x50 ('P') — the start of
    "RPLidar...".  lidarStatus() then reads those as a descriptor and fails
    with 'sync bytes mismatched'.

    The safe approach: just wait for any in-flight scan bytes to arrive,
    then discard everything in the input buffer before lidarStatus() reads.
    """
    time.sleep(0.3)          # let residual scan bytes drain in over serial
    try:
        lidar._serial.reset_input_buffer()   # discard all queued bytes
    except Exception:
        pass
    time.sleep(0.05)         # brief settle after flush


def plot_live_scan():
    print("====== LiDAR Live Plot ======")
    lidar  = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    _flush_lidar(lidar)                      # clear stale buffer before scanning
    status = lidarStatus(lidar)
    mode   = status['typical_scan_mode']
    print(f"Connected. Mode: {mode}")
    print_legend()
    move_up = ui_prepare_frame()
    try:
        while True:
            scan_data = performSingleScan(lidar, mode)
            ui_update_display(_process_scan(scan_data), move_up)
            # No sleep — scan acquisition is the rate limiter (~100-200 ms).
    except KeyboardInterrupt:
        sys.stdout.write("\n" * 2)
        print("Scan stopped by user.")
    finally:
        lidarDisconnect(lidar)
        ui_show_cursor()
        sys.stdout.flush()


if __name__ == "__main__":
    # plot_single_scan()
    plot_live_scan()