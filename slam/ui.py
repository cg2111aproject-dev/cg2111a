#!/usr/bin/env python3
"""
ui.py - Terminal map viewer using the Textual framework.

Rotated 6x3 Rectangular Robot Footprint:
- Logic: Translates grid pixels into the Robot's local coordinate system.
- Result: The blue rectangle rotates with the robot.
- 4 pixels ahead, 1 center, 1 behind, 1 each side.
"""

from __future__ import annotations

import json
import multiprocessing
import time
import math  # Required for rotation matrix

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

_MAX_PATH_HISTORY = 2000
_PATH_RECORD_DIST_MM = 50

class SlamApp(App[None]):
    CSS = """
    Screen { background: #101418; color: white; }
    #root { height: 1fr; padding: 0 0; }
    #header { height: auto; content-align: left middle; color: white; background: #1d2630; padding: 0 1; text-style: bold; }
    #map { height: 1fr; padding: 0 0; content-align: center middle; background: #0b0f14; }
    #status { height: auto; color: white; background: #182028; padding: 0 1; }
    #help { height: auto; color: #b8c4cf; background: #141b22; padding: 0 1; }
    Footer { background: #1d2630; }
    """

    BINDINGS = [
        Binding('+', 'zoom_in', 'Zoom In'), Binding('=', 'zoom_in', 'Zoom In', show=False),
        Binding('-', 'zoom_out', 'Zoom Out'), Binding('_', 'zoom_out', 'Zoom Out', show=False),
        Binding('1', 'set_zoom(0)', 'Zoom 1', show=False), Binding('2', 'set_zoom(1)', 'Zoom 2', show=False),
        Binding('3', 'set_zoom(2)', 'Zoom 3', show=False), Binding('4', 'set_zoom(3)', 'Zoom 4', show=False),
        Binding('5', 'set_zoom(4)', 'Zoom 5', show=False),
        Binding('left', 'pan_left', 'Pan Left', show=False), Binding('h', 'pan_left', 'Pan Left', show=False),
        Binding('right', 'pan_right', 'Pan Right', show=False), Binding('l', 'pan_right', 'Pan Right', show=False),
        Binding('up', 'pan_up', 'Pan Up', show=False), Binding('k', 'pan_up', 'Pan Up', show=False),
        Binding('down', 'pan_down', 'Pan Down', show=False), Binding('j', 'pan_down', 'Pan Down', show=False),
        Binding('c', 'center', 'Center'), Binding('p', 'pause_toggle', 'Pause'),
        Binding('s', 'save_map', 'Save Map'), Binding('q', 'quit', 'Quit'),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.pss = ProcessSharedState()
        self.slam_proc = multiprocessing.Process(target=run_slam_process, args=(self.pss,), name='slam-process', daemon=True)
        self.zoom_idx = DEFAULT_ZOOM
        self.pan_x_mm = self.pan_y_mm = 0.0
        self._last_render_key: tuple = ()
        self._cached_robot_visible = False
        self._path_history: list[tuple[float, float]] = []
        self._last_pose: tuple[float, float] = (0.0, 0.0)
        self._save_message: str = ''
        self._save_message_until: float = 0.0
        self._auto_follow = True
        self._follow_edge_frac = 0.25

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
        self.pss.stop_event.set()
        if self.slam_proc.is_alive(): self.slam_proc.join(timeout=3.0)
        if self.slam_proc.is_alive(): self.slam_proc.terminate()
        self.pss.cleanup()

    def action_zoom_in(self) -> None:
        self.zoom_idx = min(self.zoom_idx + 1, len(ZOOM_HALF_M) - 1)
        if ZOOM_HALF_M[self.zoom_idx] is None: self.pan_x_mm = self.pan_y_mm = 0.0

    def action_zoom_out(self) -> None:
        self.zoom_idx = max(self.zoom_idx - 1, 0)
        if ZOOM_HALF_M[self.zoom_idx] is None: self.pan_x_mm = self.pan_y_mm = 0.0

    def action_set_zoom(self, idx: str) -> None:
        idx_int = int(idx)
        if 0 <= idx_int < len(ZOOM_HALF_M):
            self.zoom_idx = idx_int
            if ZOOM_HALF_M[self.zoom_idx] is None: self.pan_x_mm = self.pan_y_mm = 0.0

    def action_pan_left(self) -> None: self.pan_y_mm += pan_step_mm(self.zoom_idx); self._auto_follow = False
    def action_pan_right(self) -> None: self.pan_y_mm -= pan_step_mm(self.zoom_idx); self._auto_follow = False
    def action_pan_up(self) -> None: self.pan_x_mm += pan_step_mm(self.zoom_idx); self._auto_follow = False
    def action_pan_down(self) -> None: self.pan_x_mm -= pan_step_mm(self.zoom_idx); self._auto_follow = False
    def action_center(self) -> None: self.pan_x_mm = self.pan_y_mm = 0.0; self._auto_follow = True
    def action_pause_toggle(self) -> None: self.pss.paused.value = not self.pss.paused.value

    def action_save_map(self) -> None:
        snapshot = self._snapshot(); ts = time.strftime('%Y%m%d_%H%M%S')
        with open(f'slam_map_{ts}.bin', 'wb') as f: f.write(snapshot['mapbytes'])
        with open(f'slam_map_{ts}.json', 'w') as f:
            json.dump({'x_mm': snapshot['x_mm'], 'y_mm': snapshot['y_mm'], 'theta_deg': snapshot['theta_deg'], 'path_history': self._path_history, 'saved_at': ts}, f)
        self._save_message = f'Saved: slam_map_{ts}.bin'; self._save_message_until = time.monotonic() + 5.0

    def action_quit(self) -> None: self.action_save_map(); self.pss.stop_event.set(); self.exit()

    def _snapshot(self) -> dict:
        return {
            'mapbytes': bytes(self.pss.shm.buf), 'x_mm': self.pss.x_mm.value, 'y_mm': self.pss.y_mm.value,
            'theta_deg': self.pss.theta_deg.value, 'valid_points': self.pss.valid_points.value, 'status_note': self.pss.get_status(),
            'rounds_seen': self.pss.rounds_seen.value, 'map_version': self.pss.map_version.value, 'pose_version': self.pss.pose_version.value,
            'connected': self.pss.connected.value, 'paused': self.pss.paused.value, 'stopped': self.pss.stopped.value, 'error_message': self.pss.get_error()
        }

    def _auto_follow_update(self, rx: float, ry: float) -> None:
        if not self._auto_follow: return
        zh = ZOOM_HALF_M[self.zoom_idx]
        if zh is None: return
        thresh = (zh * 1000.0) * (1.0 - self._follow_edge_frac)
        if abs(-self.pan_x_mm) > thresh or abs(-self.pan_y_mm) > thresh:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def _update_path_history(self, x: float, y: float) -> None:
        if math.sqrt((x - self._last_pose[0])**2 + (y - self._last_pose[1])**2) >= _PATH_RECORD_DIST_MM:
            self._path_history.append((x, y))
            if len(self._path_history) > _MAX_PATH_HISTORY: self._path_history = self._path_history[-_MAX_PATH_HISTORY:]
            self._last_pose = (x, y)

    def _render_map_text(self, snapshot: dict) -> tuple[Text, bool]:
        try:
            map_widget = self.query_one('#map', Static); region = map_widget.content_region
        except: return Text(), False

        disp_cols, disp_rows = max(20, min(region.width, MAX_RENDER_COLS)), max(8, min(region.height, MAX_RENDER_ROWS))
        rob_col, rob_row = mm_to_map_px(snapshot['x_mm'], snapshot['y_mm'])
        zh = ZOOM_HALF_M[self.zoom_idx]; px_per_m = MAP_SIZE_PIXELS / MAP_SIZE_METERS

        if zh is None: col_lo, col_hi = 0.0, float(MAP_SIZE_PIXELS); row_lo, row_hi = 0.0, float(MAP_SIZE_PIXELS)
        else:
            cx, cy = mm_to_map_px(snapshot['x_mm'] + self.pan_x_mm, snapshot['y_mm'] + self.pan_y_mm)
            half = zh * px_per_m; col_lo, col_hi, row_lo, row_hi = cx - half, cx + half, cy - half, cy + half

        col_span, row_span = max(1e-9, col_hi - col_lo), max(1e-9, row_hi - row_lo)
        robot_visible = (col_lo <= rob_col < col_hi and row_lo <= rob_row < row_hi)
        robot_sc = int((rob_col - col_lo) / col_span * disp_cols) if robot_visible else -1
        robot_sr = int((rob_row - row_lo) / row_span * disp_rows) if robot_visible else -1

        path_cells = set()
        for px, py in self._path_history:
            pc, pr = mm_to_map_px(px, py)
            if col_lo <= pc < col_hi and row_lo <= pr < row_hi:
                path_cells.add((int((pr - row_lo) / row_span * disp_rows), int((pc - col_lo) / col_span * disp_cols)))

        vis_idx = render_map_numpy(snapshot['mapbytes'], col_lo, col_hi, row_lo, row_hi, disp_cols, disp_rows)

        # PRE-CALCULATE ROTATION (Leverage for CPU speed)
        rad = math.radians(snapshot['theta_deg'] + 90)
        cos_t, sin_t = math.cos(rad), math.sin(rad)
        robot_glyph_char = robot_glyph(snapshot['theta_deg'])

        text = Text(no_wrap=True)
        for sr in range(disp_rows):
            row_data = vis_idx[sr]
            run_glyph = run_style = ''
            for sc in range(disp_cols):
                # Rotated rectangle logic
                is_robot_rect = False
                if robot_visible:
                    dx, dy = sc - robot_sc, sr - robot_sr
                    local_x = dx * cos_t + dy * sin_t
                    local_y = -dx * sin_t + dy * cos_t
                    # Bounds: Width +/- 1.5, Front -4.5, Back +1.5
                    if (-1.5 < local_x < 1.5) and (-4.5 < local_y < 1.5):
                        is_robot_rect = True

                if is_robot_rect:
                    if run_glyph: text.append(run_glyph, style=run_style); run_glyph = ''
                    text.append(robot_glyph_char, style=_STYLE_ROBOT); continue

                if (sr, sc) in path_cells and int(row_data[sc]) >= 2:
                    if run_glyph: text.append(run_glyph, style=run_style); run_glyph = ''
                    text.append(_GLYPH_PATH, style=_STYLE_PATH); continue

                _, glyph, style = _VIS_TABLE[int(row_data[sc])]
                if style == run_style: run_glyph += glyph
                else:
                    if run_glyph: text.append(run_glyph, style=run_style)
                    run_glyph, run_style = glyph, style

            if run_glyph: text.append(run_glyph, style=run_style)
            if sr != disp_rows - 1: text.append('\n')
        return text, robot_visible

    def _refresh_view(self) -> None:
        try: h, m, s, help_w = self.query_one('#header'), self.query_one('#map'), self.query_one('#status'), self.query_one('#help')
        except: return
        snap = self._snapshot(); self._update_path_history(snap['x_mm'], snap['y_mm']); self._auto_follow_update(snap['x_mm'], snap['y_mm'])
        state = 'PAUSED' if snap['paused'] else 'LIVE'
        if snap['error_message']: state = 'ERROR'
        zh = ZOOM_HALF_M[self.zoom_idx]
        view_str = f'full {MAP_SIZE_METERS}m' if zh is None else f'close {zh*2}m'
        pan_text = 'centered' if abs(self.pan_x_mm) < 0.5 else f'{self.pan_x_mm/1000:+.2f}m'
        h.update(f'SLAM Map | {view_str} | Zoom {self.zoom_idx+1} | {state} | Follow: {"ON" if self._auto_follow else "OFF"}')
        
        region = m.content_region
        render_key = (snap['map_version'], snap['pose_version'], self.zoom_idx, round(self.pan_x_mm), round(self.pan_y_mm), region.width, region.height, len(self._path_history))
        if render_key != self._last_render_key:
            mt, rv = self._render_map_text(snap); self._cached_robot_visible, self._last_render_key = rv, render_key; m.update(mt)

        s_line = f'Pose x={snap["x_mm"]:4.0f} y={snap["y_mm"]:4.0f} th={snap["theta_deg"]:+5.1f} | pan={pan_text}'
        if not self._cached_robot_visible: s_line += ' | OFF-SCREEN'
        if snap['error_message']: s_line += f' | ERROR: {snap["error_message"]}'
        s.update(s_line)
        help_w.update('Legend: [bold white on red]█ wall[/] [green]░ free[/] [bold bright_cyan]◉ robot[/] | c: center p: pause q: quit')

def run() -> None: SlamApp().run()
