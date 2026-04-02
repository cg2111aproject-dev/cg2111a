#!/usr/bin/env python3
"""
ui.py - Terminal map viewer using the Textual framework.

Displays the BreezySLAM occupancy map as a colour Unicode block-character
grid in the terminal.  The map updates in real time as the robot moves.

Keyboard controls
-----------------
  + / =     Zoom in (show a smaller area in more detail)
  - / _     Zoom out (zoom level 1 shows the full map)
  1-5       Jump directly to zoom levels 1-5
  arrows    Pan the zoomed view
  h j k l   Pan (vi-style: left, down, up, right)
  c         Re-centre the view on the robot
  p         Pause / resume SLAM updates
  s         Save map to file (also auto-saves on quit)
  q         Quit (also Ctrl-C)

Map legend
----------
  white on red  full block  wall / confirmed obstacle
  bold red      dark shade  near a wall
  bold yellow   mid shade   frontier (mixed free/occupied)
  dark grey     middle dot  unknown / not yet visited
  green         light shade probable free space
  bright green  white dot   confirmed free space
  cyan          fisheye     robot estimated position
  magenta       small dot   path history
"""

from __future__ import annotations

import json
import multiprocessing
import time

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.widgets import Footer, Static

from settings import (
    MAP_SIZE_PIXELS, MAP_SIZE_METERS,
    ZOOM_HALF_M, DEFAULT_ZOOM,
    UI_REFRESH_HZ, MAX_RENDER_COLS, MAX_RENDER_ROWS,
)
from shared_state import ProcessSharedState
from slam_process import run_slam_process
from renderer import (
    _VIS_TABLE, _STYLE_ROBOT,
    _GLYPH_WALL, _STYLE_WALL,
    _GLYPH_WALL_SOFT, _STYLE_WALL_SOFT,
    _GLYPH_FRONTIER, _STYLE_FRONTIER,
    _GLYPH_UNKNOWN, _STYLE_UNKNOWN,
    _GLYPH_FREE_CLEAR, _STYLE_FREE_CLEAR,
    _GLYPH_PATH, _STYLE_PATH,
    mm_to_map_px, pan_step_mm, robot_glyph, render_map_numpy,
)

# Maximum number of path history points to keep in memory.
_MAX_PATH_HISTORY = 2000
# Minimum distance (mm) the robot must move before a new path point is recorded.
_PATH_RECORD_DIST_MM = 50


class SlamApp(App[None]):
    """Full-screen Textual application that displays the SLAM map."""

    CSS = """
    Screen {
        background: #101418;
        color: white;
    }
    #root {
        height: 1fr;
        padding: 0 0;
    }
    #header {
        height: auto;
        content-align: left middle;
        color: white;
        background: #1d2630;
        padding: 0 1;
        text-style: bold;
    }
    #map {
        height: 1fr;
        padding: 0 0;
        content-align: center middle;
        background: #0b0f14;
    }
    #status {
        height: auto;
        color: white;
        background: #182028;
        padding: 0 1;
    }
    #help {
        height: auto;
        color: #b8c4cf;
        background: #141b22;
        padding: 0 1;
    }
    Footer {
        background: #1d2630;
    }
    """

    BINDINGS = [
        Binding('+',     'zoom_in',      'Zoom In'),
        Binding('=',     'zoom_in',      'Zoom In',   show=False),
        Binding('-',     'zoom_out',     'Zoom Out'),
        Binding('_',     'zoom_out',     'Zoom Out',  show=False),
        Binding('1',     'set_zoom(0)',  'Zoom 1',    show=False),
        Binding('2',     'set_zoom(1)',  'Zoom 2',    show=False),
        Binding('3',     'set_zoom(2)',  'Zoom 3',    show=False),
        Binding('4',     'set_zoom(3)',  'Zoom 4',    show=False),
        Binding('5',     'set_zoom(4)',  'Zoom 5',    show=False),
        Binding('left',  'pan_left',    'Pan Left',  show=False),
        Binding('h',     'pan_left',    'Pan Left',  show=False),
        Binding('right', 'pan_right',   'Pan Right', show=False),
        Binding('l',     'pan_right',   'Pan Right', show=False),
        Binding('up',    'pan_up',      'Pan Up',    show=False),
        Binding('k',     'pan_up',      'Pan Up',    show=False),
        Binding('down',  'pan_down',    'Pan Down',  show=False),
        Binding('j',     'pan_down',    'Pan Down',  show=False),
        Binding('c',     'center',      'Center'),
        Binding('p',     'pause_toggle','Pause'),
        Binding('s',     'save_map',    'Save Map'),
        Binding('q',     'quit',        'Quit'),
    ]

    def __init__(self) -> None:
        super().__init__()
        # Shared state object passed to the SLAM process.
        self.pss = ProcessSharedState()
        # SLAM runs in a separate child process so it has its own GIL.
        self.slam_proc = multiprocessing.Process(
            target=run_slam_process,
            args=(self.pss,),
            name='slam-process',
            daemon=True,
        )
        self.zoom_idx = DEFAULT_ZOOM
        self.pan_x_mm = 0.0
        self.pan_y_mm = 0.0
        # Cache the last render key so we skip redraws when nothing changed.
        self._last_render_key: tuple = ()
        self._cached_robot_visible = False

        # Path history: list of (x_mm, y_mm) positions the robot has visited.
        self._path_history: list[tuple[float, float]] = []
        self._last_pose: tuple[float, float] = (0.0, 0.0)

        # Last save message to show in status bar.
        self._save_message: str = ''
        self._save_message_until: float = 0.0

        # Auto-follow: pan the view when the robot nears the edge.
        # Disabled temporarily when the user manually pans; press c to re-enable.
        self._auto_follow: bool = True
        # How close to the edge (as a fraction of view half-width) before panning.
        self._follow_edge_frac: float = 0.25

    def compose(self) -> ComposeResult:
        with Vertical(id='root'):
            yield Static(id='header')
            yield Static(id='map')
            yield Static(id='status')
            yield Static(id='help')
        yield Footer()

    def on_mount(self) -> None:
        self.slam_proc.start()
        self.set_interval(1.0 / UI_REFRESH_HZ, self._refresh_view)

    def on_unmount(self) -> None:
        # Signal the SLAM process to stop, then wait for it to exit cleanly.
        self.pss.stop_event.set()
        if self.slam_proc.is_alive():
            self.slam_proc.join(timeout=3.0)
        if self.slam_proc.is_alive():
            self.slam_proc.terminate()
        self.pss.cleanup()

    # -----------------------------------------------------------------------
    # Keyboard actions
    # -----------------------------------------------------------------------

    def action_zoom_in(self) -> None:
        self.zoom_idx = min(self.zoom_idx + 1, len(ZOOM_HALF_M) - 1)
        if ZOOM_HALF_M[self.zoom_idx] is None:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def action_zoom_out(self) -> None:
        self.zoom_idx = max(self.zoom_idx - 1, 0)
        if ZOOM_HALF_M[self.zoom_idx] is None:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def action_set_zoom(self, idx: str) -> None:
        idx_int = int(idx)
        if 0 <= idx_int < len(ZOOM_HALF_M):
            self.zoom_idx = idx_int
            if ZOOM_HALF_M[self.zoom_idx] is None:
                self.pan_x_mm = self.pan_y_mm = 0.0

    def action_pan_left(self) -> None:
        self.pan_y_mm += pan_step_mm(self.zoom_idx)
        self._auto_follow = False

    def action_pan_right(self) -> None:
        self.pan_y_mm -= pan_step_mm(self.zoom_idx)
        self._auto_follow = False

    def action_pan_up(self) -> None:
        self.pan_x_mm += pan_step_mm(self.zoom_idx)
        self._auto_follow = False

    def action_pan_down(self) -> None:
        self.pan_x_mm -= pan_step_mm(self.zoom_idx)
        self._auto_follow = False

    def action_center(self) -> None:
        self.pan_x_mm = self.pan_y_mm = 0.0
        self._auto_follow = True

    def action_pause_toggle(self) -> None:
        self.pss.paused.value = not self.pss.paused.value

    def action_save_map(self) -> None:
        """Save the current map and path history to disk."""
        snapshot = self._snapshot()
        ts = time.strftime('%Y%m%d_%H%M%S')

        # Save raw map bytes — can be reloaded with view_map.py
        map_path = f'slam_map_{ts}.bin'
        with open(map_path, 'wb') as f:
            f.write(snapshot['mapbytes'])

        # Save pose and path history as JSON alongside the .bin file
        meta_path = f'slam_map_{ts}.json'
        with open(meta_path, 'w') as f:
            json.dump({
                'x_mm':         snapshot['x_mm'],
                'y_mm':         snapshot['y_mm'],
                'theta_deg':    snapshot['theta_deg'],
                'path_history': self._path_history,
                'saved_at':     ts,
            }, f)

        self._save_message = f'Saved: {map_path}'
        self._save_message_until = time.monotonic() + 5.0

    def action_quit(self) -> None:
        # Autosave before exit so the map is never lost.
        self.action_save_map()
        self.pss.stop_event.set()
        self.exit()

    # -----------------------------------------------------------------------
    # Read a snapshot from shared memory
    # -----------------------------------------------------------------------

    def _snapshot(self) -> dict:
        """Read all shared state into a plain dict for safe single-threaded use."""
        error = self.pss.get_error()
        return {
            'mapbytes':      bytes(self.pss.shm.buf),
            'x_mm':          self.pss.x_mm.value,
            'y_mm':          self.pss.y_mm.value,
            'theta_deg':     self.pss.theta_deg.value,
            'valid_points':  self.pss.valid_points.value,
            'status_note':   self.pss.get_status(),
            'rounds_seen':   self.pss.rounds_seen.value,
            'map_version':   self.pss.map_version.value,
            'pose_version':  self.pss.pose_version.value,
            'connected':     self.pss.connected.value,
            'paused':        self.pss.paused.value,
            'stopped':       self.pss.stopped.value,
            'error_message': error if error else None,
        }

    # -----------------------------------------------------------------------
    # Auto-follow: pan when robot nears the edge of the view
    # -----------------------------------------------------------------------

    def _auto_follow_update(self, robot_x_mm: float, robot_y_mm: float) -> None:
        """Re-centre the view when the robot gets close to the view edge.

        The view is always centred at (robot_x_mm + pan_x_mm, robot_y_mm + pan_y_mm).
        So the robot distance from view centre is simply (-pan_x, -pan_y).
        When that exceeds threshold_mm in either axis, snap pan back to 0.

        Only active on zoomed views (zoom 2-5). Full map shows everything.
        """
        if not self._auto_follow:
            return
        zoom_half_m = ZOOM_HALF_M[self.zoom_idx]
        if zoom_half_m is None:
            return  # full map always shows everything

        half_mm = zoom_half_m * 1000.0
        threshold_mm = half_mm * (1.0 - self._follow_edge_frac)

        # Robot offset from view centre = -pan offset
        robot_from_centre_x = -self.pan_x_mm
        robot_from_centre_y = -self.pan_y_mm

        if (abs(robot_from_centre_x) > threshold_mm or
                abs(robot_from_centre_y) > threshold_mm):
            self.pan_x_mm = 0.0
            self.pan_y_mm = 0.0

    # -----------------------------------------------------------------------
    # Path history tracking
    # -----------------------------------------------------------------------

    def _update_path_history(self, x_mm: float, y_mm: float) -> None:
        """Record the robot's position if it has moved far enough."""
        dx = x_mm - self._last_pose[0]
        dy = y_mm - self._last_pose[1]
        if (dx * dx + dy * dy) ** 0.5 >= _PATH_RECORD_DIST_MM:
            self._path_history.append((x_mm, y_mm))
            # Keep history bounded so memory doesn't grow forever.
            if len(self._path_history) > _MAX_PATH_HISTORY:
                self._path_history = self._path_history[-_MAX_PATH_HISTORY:]
            self._last_pose = (x_mm, y_mm)

    # -----------------------------------------------------------------------
    # Map rendering
    # -----------------------------------------------------------------------

    def _render_map_text(self, snapshot: dict) -> tuple[Text, bool]:
        """Render the occupancy map as a Rich Text object.

        Returns (text, robot_visible).
        """
        try:
            map_widget = self.query_one('#map', Static)
            region = map_widget.content_region
        except Exception:
            return Text(), False

        disp_cols = max(20, min(region.width,  MAX_RENDER_COLS))
        disp_rows = max(8,  min(region.height, MAX_RENDER_ROWS))

        mapbytes    = snapshot['mapbytes']
        robot_x_mm  = snapshot['x_mm']
        robot_y_mm  = snapshot['y_mm']
        px_per_m    = MAP_SIZE_PIXELS / MAP_SIZE_METERS

        rob_col, rob_row = mm_to_map_px(robot_x_mm, robot_y_mm)

        # Compute the pixel bounds of the view window.
        zoom_half_m = ZOOM_HALF_M[self.zoom_idx]
        if zoom_half_m is None:
            col_lo, col_hi = 0.0, float(MAP_SIZE_PIXELS)
            row_lo, row_hi = 0.0, float(MAP_SIZE_PIXELS)
        else:
            cx, cy = mm_to_map_px(
                robot_x_mm + self.pan_x_mm,
                robot_y_mm + self.pan_y_mm,
            )
            half = zoom_half_m * px_per_m
            col_lo = cx - half
            col_hi = cx + half
            row_lo = cy - half
            row_hi = cy + half

        col_span = max(1e-9, col_hi - col_lo)
        row_span = max(1e-9, row_hi - row_lo)

        # Determine robot display position.
        robot_visible = (col_lo <= rob_col < col_hi and
                         row_lo <= rob_row < row_hi)
        if robot_visible:
            robot_sc = max(0, min(disp_cols - 1,
                int((rob_col - col_lo) / col_span * disp_cols)))
            robot_sr = max(0, min(disp_rows - 1,
                int((rob_row - row_lo) / row_span * disp_rows)))
        else:
            robot_sc = robot_sr = -1

        # Build set of path history cells within the current view.
        path_cells: set[tuple[int, int]] = set()
        for px, py in self._path_history:
            pc, pr = mm_to_map_px(px, py)
            if col_lo <= pc < col_hi and row_lo <= pr < row_hi:
                sc_ = max(0, min(disp_cols - 1,
                    int((pc - col_lo) / col_span * disp_cols)))
                sr_ = max(0, min(disp_rows - 1,
                    int((pr - row_lo) / row_span * disp_rows)))
                path_cells.add((sr_, sc_))

        # Downsample the map region to display resolution.
        vis_idx = render_map_numpy(
            mapbytes, col_lo, col_hi, row_lo, row_hi, disp_cols, disp_rows,
        )

        # Build the Rich Text object row by row.
        text = Text(no_wrap=True)
        for sr in range(disp_rows):
            row_data = vis_idx[sr]
            run_glyph = ''
            run_style = ''

            for sc in range(disp_cols):
                # Robot marker takes priority.
                if robot_visible and (robot_sr - 5 <= sr <= robot_sr + 1) and (robot_sc - 1 <= sc <= robot_sr + 1):
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                        run_glyph = ''
                    text.append(
                        robot_glyph(snapshot['theta_deg']),
                        style=_STYLE_ROBOT,
                    )
                    continue

                # Path history dots drawn over free/unknown cells only.
                if (sr, sc) in path_cells:
                    vis = int(row_data[sc])
                    # Only draw path dot if the cell is not a wall
                    # (avoid hiding wall information with path dots).
                    if vis >= 2:
                        if run_glyph:
                            text.append(run_glyph, style=run_style)
                            run_glyph = ''
                        text.append(_GLYPH_PATH, style=_STYLE_PATH)
                        continue

                _, glyph, style = _VIS_TABLE[int(row_data[sc])]

                if style == run_style:
                    run_glyph += glyph
                else:
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                    run_glyph = glyph
                    run_style = style

            if run_glyph:
                text.append(run_glyph, style=run_style)
            if sr != disp_rows - 1:
                text.append('\n')

        return text, robot_visible

    # -----------------------------------------------------------------------
    # Periodic refresh
    # -----------------------------------------------------------------------

    def _refresh_view(self) -> None:
        """Called by Textual at UI_REFRESH_HZ; reads shared state and redraws."""
        try:
            header        = self.query_one('#header',  Static)
            map_widget    = self.query_one('#map',     Static)
            status_widget = self.query_one('#status',  Static)
            help_widget   = self.query_one('#help',    Static)
        except Exception:
            return

        snapshot = self._snapshot()

        # Update path history.
        self._update_path_history(snapshot['x_mm'], snapshot['y_mm'])

        # Auto-follow: pan to keep robot in view.
        self._auto_follow_update(snapshot['x_mm'], snapshot['y_mm'])

        # Determine the overall state label shown in the header.
        state = 'PAUSED' if snapshot['paused'] else 'LIVE'
        if snapshot['error_message']:
            state = 'ERROR'
        elif snapshot['stopped'] and not snapshot['connected']:
            state = 'STOPPED'

        # Describe the current view window.
        half_m = ZOOM_HALF_M[self.zoom_idx]
        if half_m is None:
            view_str = (f'full map {MAP_SIZE_METERS:.1f}m x '
                        f'{MAP_SIZE_METERS:.1f}m')
            pan_text = 'centered'
        else:
            view_str = f'close-up {half_m * 2:.1f}m x {half_m * 2:.1f}m'
            if abs(self.pan_x_mm) < 0.5 and abs(self.pan_y_mm) < 0.5:
                pan_text = 'centered'
            else:
                pan_text = (f'{self.pan_x_mm / 1000:+.2f},'
                            f'{self.pan_y_mm / 1000:+.2f}m')

        header.update(
            f'SLAM Map | View: {view_str} | '
            f'Zoom {self.zoom_idx + 1}/{len(ZOOM_HALF_M)} | {state} | '
            f'Path pts: {len(self._path_history)} | '
            f'Follow: {"ON" if self._auto_follow else "OFF (c to resume)"}'
        )

        # Only re-render the map when something that affects the image changed.
        region = map_widget.content_region
        render_key = (
            snapshot['map_version'],
            snapshot['pose_version'],
            self.zoom_idx,
            round(self.pan_x_mm, 0),
            round(self.pan_y_mm, 0),
            region.width,
            region.height,
            len(self._path_history),
        )
        if render_key != self._last_render_key:
            map_text, robot_visible = self._render_map_text(snapshot)
            self._cached_robot_visible = robot_visible
            self._last_render_key = render_key
            map_widget.update(map_text)

        robot_visible = self._cached_robot_visible

        # Status bar.
        status_line = (
            f'Pose x={snapshot["x_mm"]:6.0f}mm  '
            f'y={snapshot["y_mm"]:6.0f}mm  '
            f'th={snapshot["theta_deg"]:+6.1f}deg | '
            f'valid={snapshot["valid_points"]:3d} | '
            f'pan={pan_text} | '
            f'{snapshot["status_note"]}'
        )
        if not robot_visible:
            status_line += ' | robot off-screen'
        if snapshot['error_message']:
            status_line += f' | ERROR: {snapshot["error_message"]}'
        # Show save message for 5 seconds after saving.
        if self._save_message and time.monotonic() < self._save_message_until:
            status_line += f' | \u2713 {self._save_message}'
        status_widget.update(status_line)

        # Help bar: map legend and key hints.
        help_widget.update(
            'Legend: '
            f'[{_STYLE_WALL}]{_GLYPH_WALL} wall[/]  '
            f'[{_STYLE_WALL_SOFT}]{_GLYPH_WALL_SOFT} obstacle[/]  '
            f'[{_STYLE_FRONTIER}]{_GLYPH_FRONTIER} mixed[/]  '
            f'[{_STYLE_UNKNOWN}]{_GLYPH_UNKNOWN} unknown[/]  '
            f'[{_STYLE_FREE_CLEAR}]{_GLYPH_FREE_CLEAR} free[/]  '
            f'[{_STYLE_ROBOT}]\u25c9 robot[/]  '
            f'[{_STYLE_PATH}]{_GLYPH_PATH} path[/]  |  '
            'Keys: +/- zoom  1-5 jump  arrows/hjkl pan (disables follow)  '
            'c center+follow  p pause  s save  q quit'
        )


def run() -> None:
    """Launch the SLAM map viewer.  Called from slam.py."""
    try:
        import rich   # noqa: F401
        import textual  # noqa: F401
    except ImportError:
        print('[slam] ERROR: textual is not installed.')
        print('  Run:  bash ../install_slam.sh  or activate the environment.')
        raise SystemExit(1)

    SlamApp().run()
