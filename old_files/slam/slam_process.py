#!/usr/bin/env python3
"""
slam_process.py - SLAM background process.

This module contains the function that runs in a dedicated child process to
perform SLAM.  Running SLAM in a separate process gives it its own Python GIL,
so heavy LIDAR reading and map-building never stalls the terminal UI.

The process reads LIDAR scans, resamples them into fixed-size angle bins, and
feeds them into BreezySLAM's RMHC_SLAM algorithm.  The resulting robot pose
and occupancy map are written into a ProcessSharedState object that the UI
process can read at any time.

Scan resampling
---------------
The RPLidar reports measurements at irregular angles.  BreezySLAM expects a
fixed array of SCAN_SIZE evenly-spaced readings.  The resampling step bins raw
readings by their rounded angle (0-359 degrees) and averages any multiple
readings that fall in the same bin.  Bins with no readings are filled with
MAX_DISTANCE_MM (treated as "no obstacle detected").

Smoother map updates
--------------------
The map is copied to shared memory at MAP_UPDATE_INTERVAL.  To reduce CPU
overhead, the copy is skipped when the map content has not actually changed
since the last write (detected by comparing a fast hash of the mapbytes).
"""

from __future__ import annotations

import time
from typing import Optional

from settings import (
    SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM,
    MAP_SIZE_PIXELS, MAP_SIZE_METERS, HOLE_WIDTH_MM, MAP_QUALITY,
    LIDAR_OFFSET_DEG, MIN_VALID_POINTS, INITIAL_ROUNDS_SKIP,
    MAP_UPDATE_INTERVAL,
)
from shared_state import ProcessSharedState


# Pre-compute the angle array once so we do not recreate it every scan.
_SCAN_ANGLES: list[float] = [
    float(i * DETECTION_ANGLE / SCAN_SIZE)
    for i in range(SCAN_SIZE)
]


def _resample_scan(
    raw_angles: list[float],
    raw_distances: list[float],
) -> tuple[list[int], int]:
    """Resample raw LIDAR readings into SCAN_SIZE equal-angle bins.

    The RPLidar reports angles in clockwise order (0 = forward, increasing
    clockwise when viewed from above).  BreezySLAM expects counter-clockwise
    angles (0 = forward, increasing CCW).  We negate the raw angle to convert
    from CW to CCW before applying LIDAR_OFFSET_DEG.

    Parameters
    ----------
    raw_angles    : angle for each raw measurement, in degrees (CW, RPLidar)
    raw_distances : distance for each raw measurement, in mm

    Returns
    -------
    scan_distances : list of SCAN_SIZE integer distances (mm)
    valid          : number of bins that contained at least one non-zero reading
                     below MAX_DISTANCE_MM
    """
    bin_sums = [0.0] * SCAN_SIZE
    bin_counts = [0] * SCAN_SIZE

    for angle, dist in zip(raw_angles, raw_distances):
        if dist <= 0:
            continue
        ccw_angle = -angle + LIDAR_OFFSET_DEG
        bin_idx = int(round(ccw_angle)) % SCAN_SIZE
        bin_sums[bin_idx] += dist
        bin_counts[bin_idx] += 1

    scan_distances: list[int] = []
    valid = 0
    for i in range(SCAN_SIZE):
        if bin_counts[i] > 0:
            avg = bin_sums[i] / bin_counts[i]
            if avg >= MAX_DISTANCE_MM:
                scan_distances.append(MAX_DISTANCE_MM)
            else:
                scan_distances.append(int(avg))
                valid += 1
        else:
            scan_distances.append(MAX_DISTANCE_MM)

    return scan_distances, valid


def run_slam_process(pss: ProcessSharedState) -> None:
    """Entry point for the SLAM child process.

    Connects to the LIDAR, initialises BreezySLAM, then loops:
      1. Read one full 360-degree LIDAR scan.
      2. Resample it into SCAN_SIZE equal-angle bins.
      3. Feed the resampled scan to RMHC_SLAM.update().
      4. Read back the updated pose and (at a throttled rate) the map.
      5. Write both into pss so the UI can pick them up.

    The map copy is skipped when the content is unchanged (hash comparison)
    to reduce unnecessary shared-memory writes and UI redraws.

    The loop exits when pss.stop_event is set (by the UI on quit).
    """
    try:
        from breezyslam.algorithms import RMHC_SLAM
        from breezyslam.sensors import Laser
    except ImportError:
        pss.set_error('BreezySLAM not installed. Run: bash install_slam.sh')
        pss.stopped.value = True
        return

    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
    print(f"[DEBUG] sys.path[0] = {_sys.path[0]}", flush=True)
    print(f"[DEBUG] lidar.py exists = {_os.path.exists(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), 'lidar.py'))}", flush=True)
    try:
        import lidar as lidar_driver
    except ImportError as e:
        print(f"[DEBUG] ImportError: {e}", flush=True)
        pss.set_error('lidar.py not found in the slam/ directory')
        pss.stopped.value = True
        return

    lidar = lidar_driver.connect()
    if lidar is None:
        from settings import LIDAR_PORT
        pss.set_error(f'Could not connect to LIDAR on {LIDAR_PORT}')
        pss.stopped.value = True
        return

    scan_mode = lidar_driver.get_scan_mode(lidar)

    laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM)
    slam = RMHC_SLAM(
        laser,
        MAP_SIZE_PIXELS,
        MAP_SIZE_METERS,
        hole_width_mm=HOLE_WIDTH_MM,
        map_quality=MAP_QUALITY,
    )
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    pss.set_status(f'connected (mode {scan_mode})')
    pss.connected.value = True

    previous_distances: Optional[list[int]] = None
    round_num = 0
    last_map_update = time.monotonic()
    # Hash of the last map bytes we wrote to shared memory.
    # We skip the write if the map content hasn't changed.
    last_map_hash: Optional[int] = None

    try:
        for raw_angles, raw_distances in lidar_driver.scan_rounds(lidar, scan_mode):
            if pss.stop_event.is_set():
                break

            round_num += 1
            pss.rounds_seen.value = round_num

            if round_num <= INITIAL_ROUNDS_SKIP:
                pss.valid_points.value = 0
                pss.set_status(f'warming up {round_num}/{INITIAL_ROUNDS_SKIP}')
                continue

            if pss.paused.value:
                pss.set_status('paused')
                continue

            scan_distances, valid = _resample_scan(raw_angles, raw_distances)
            pss.valid_points.value = valid

            if valid >= MIN_VALID_POINTS:
                slam.update(scan_distances, scan_angles_degrees=_SCAN_ANGLES)
                previous_distances = list(scan_distances)
                note = f'live ({valid} pts)'
            elif previous_distances is not None:
                slam.update(previous_distances, scan_angles_degrees=_SCAN_ANGLES)
                note = f'reusing previous ({valid} pts)'
            else:
                pss.set_status(f'waiting ({valid} pts)')
                continue

            # Update pose on every scan for smooth position tracking.
            x_mm, y_mm, theta_deg = slam.getpos()
            pss.x_mm.value = x_mm
            pss.y_mm.value = y_mm
            pss.theta_deg.value = theta_deg
            pss.pose_version.value += 1

            # Copy map to shared memory at the configured rate,
            # but only if the content has actually changed.
            now = time.monotonic()
            if now - last_map_update >= MAP_UPDATE_INTERVAL:
                slam.getmap(mapbytes)
                new_hash = hash(bytes(mapbytes))
                if new_hash != last_map_hash:
                    pss.shm.buf[:len(mapbytes)] = mapbytes
                    pss.map_version.value += 1
                    last_map_hash = new_hash
                last_map_update = now

            pss.set_status(note)

    except Exception as exc:
        pss.set_error(f'SLAM process error: {exc}')
    finally:
        try:
            lidar_driver.disconnect(lidar)
        except Exception:
            pass
        pss.connected.value = False
        pss.stopped.value = True