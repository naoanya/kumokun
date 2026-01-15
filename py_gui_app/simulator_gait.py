import os
import sys
import math
import threading
import time
from collections import deque

import numpy as np
import tkinter as tk
from tkinter import ttk

# Ensure local module import works
SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Add py_utils to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
py_utils_path = os.path.join(os.path.dirname(current_dir), 'py_utils')
if py_utils_path not in sys.path:
    sys.path.append(py_utils_path)

from hexapod_gait import HexapodGait, TWO_PI, State, DT

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


def convex_hull(points):
    """Compute 2D convex hull (Andrew's monotone chain). Returns list of points in CCW order."""
    pts = sorted(points)
    if len(pts) <= 1:
        return pts

    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenate lower and upper to form full hull; omit last point of each (it's repeated)
    hull = lower[:-1] + upper[:-1]
    return hull

# Simple GUI to visualize hexapod_gait outputs in 2D (XY) + Z waveforms

FRAME_MS = 10  # update interval for GUI in ms (must match hexapod_gait DT)
HISTORY_MS = 1000  # time window for Z waveform (ms)
HIST_LEN = max(10, HISTORY_MS // FRAME_MS)

# Fixed 2D plot size (half-range in meters)
PLOT_LATERAL_RANGE = 0.3  # lateral half-range (Y axis, horizontal)
PLOT_FORWARD_RANGE = 0.3  # forward half-range (X axis, vertical)

LEG_NAMES = ["LF", "LM", "LR", "RF", "RM", "RR"]

COLORS = ['r', 'g', 'b', 'c', 'm', 'y']


class App:
    def __init__(self, root):
        self.root = root
        root.title('Hexapod Gait Visualizer (2D + Z waveforms)')

        self.gait = HexapodGait()

        # Controls frame
        ctrl = ttk.Frame(root)
        ctrl.pack(side=tk.LEFT, fill=tk.Y, padx=6, pady=6)

        ttk.Label(ctrl, text='Speed').pack()
        self.speed_var = tk.DoubleVar(value=0.5)
        ttk.Scale(ctrl, from_=0.0, to=1.0, orient=tk.HORIZONTAL, variable=self.speed_var).pack()
        self.speed_val_var = tk.StringVar(value=f"{self.speed_var.get():.3f}")
        ttk.Label(ctrl, textvariable=self.speed_val_var).pack()

        ttk.Label(ctrl, text='Direction (rad)').pack()
        self.dir_var = tk.DoubleVar(value=0.0)
        ttk.Scale(ctrl, from_=-math.pi, to=math.pi, orient=tk.HORIZONTAL, variable=self.dir_var).pack()
        self.dir_val_var = tk.StringVar(value=f"{self.dir_var.get():.3f}")
        ttk.Label(ctrl, textvariable=self.dir_val_var).pack()

        ttk.Label(ctrl, text='Rotation').pack()
        self.rot_var = tk.DoubleVar(value=0.0)
        ttk.Scale(ctrl, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, variable=self.rot_var).pack()
        self.rot_val_var = tk.StringVar(value=f"{self.rot_var.get():.3f}")
        ttk.Label(ctrl, textvariable=self.rot_val_var).pack()

        # Max forward/swing speed controls (m/s)
        ttk.Label(ctrl, text='Max Forward Speed (m/s)').pack(pady=(8,0))
        self.max_forward_speed_var = tk.DoubleVar(value=getattr(self.gait, 'max_forward_speed', 0.2))
        ttk.Scale(ctrl, from_=0.0, to=1.0, orient=tk.HORIZONTAL, variable=self.max_forward_speed_var).pack()
        self.max_forward_speed_val_var = tk.StringVar(value=f"{self.max_forward_speed_var.get():.3f}")
        ttk.Label(ctrl, textvariable=self.max_forward_speed_val_var).pack()

        ttk.Label(ctrl, text='Max Swing Speed (m/s)').pack(pady=(6,0))
        self.max_swing_speed_var = tk.DoubleVar(value=getattr(self.gait, 'max_swing_speed', getattr(self.gait, 'max_forward_speed', 0.2)))
        ttk.Scale(ctrl, from_=0.0, to=1.0, orient=tk.HORIZONTAL, variable=self.max_swing_speed_var).pack()
        self.max_swing_speed_val_var = tk.StringVar(value=f"{self.max_swing_speed_var.get():.3f}")
        ttk.Label(ctrl, textvariable=self.max_swing_speed_val_var).pack()

        # Stance fraction control (proportion of cycle spent in stance)
        ttk.Label(ctrl, text='Stance Fraction (0..1)').pack(pady=(6,0))
        self.stance_var = tk.DoubleVar(value=getattr(self.gait, 'stance_fraction', 0.5))
        ttk.Scale(ctrl, from_=0.1, to=0.9, orient=tk.HORIZONTAL, variable=self.stance_var).pack()
        self.stance_val_var = tk.StringVar(value=f"{self.stance_var.get():.3f}")
        ttk.Label(ctrl, textvariable=self.stance_val_var).pack()

        # Turn parameter controls
        ttk.Label(ctrl, text='Max Turn Rate (rad/s)').pack(pady=(8,0))
        self.turn_rate_var = tk.DoubleVar(value=getattr(self.gait, 'max_turn_rate', 1.0))
        ttk.Scale(ctrl, from_=0.0, to=3.0, orient=tk.HORIZONTAL, variable=self.turn_rate_var).pack()
        self.turn_rate_val_var = tk.StringVar(value=f"{self.turn_rate_var.get():.3f}")
        ttk.Label(ctrl, textvariable=self.turn_rate_val_var).pack()

        # Gait selection (set at start; readonly, disabled while running)
        ttk.Label(ctrl, text='Gait Pattern').pack(pady=(8,0))
        self.gait_var = tk.StringVar(value='tripod')
        self.gait_combo = ttk.Combobox(ctrl, values=['tripod', 'wave', 'ripple'], textvariable=self.gait_var, state='readonly')
        self.gait_combo.pack()

        self.start_btn = ttk.Button(ctrl, text='Start', command=self.start)
        self.start_btn.pack(fill=tk.X, pady=(10,2))
        self.stop_btn = ttk.Button(ctrl, text='Stop', command=self.stop, state=tk.DISABLED)
        self.stop_btn.pack(fill=tk.X)

        # State display
        ttk.Separator(ctrl, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(10,5))
        ttk.Label(ctrl, text='State:').pack()
        self.state_var = tk.StringVar(value='IDLE')
        self.state_label = ttk.Label(ctrl, textvariable=self.state_var, font=('TkDefaultFont', 12, 'bold'))
        self.state_label.pack()
        # FPS display
        ttk.Label(ctrl, text='FPS:').pack()
        self.fps_var = tk.StringVar(value='0.0')
        self.fps_label = ttk.Label(ctrl, textvariable=self.fps_var, font=('TkDefaultFont', 10, 'bold'))
        self.fps_label.pack()

        # Canvas / Matplotlib arranged with GridSpec:
        # left column: big XY (spanning all rows). Right columns: per-leg rows (phase, Z) for each of 6 legs.
        fig = Figure(figsize=(12, 8))
        gs = fig.add_gridspec(nrows=6, ncols=3, wspace=0.4, hspace=0.6)

        # Big XY axis on the left spanning all rows
        self.ax_xy = fig.add_subplot(gs[:, 0])
        self.ax_xy.set_aspect('equal', adjustable='box')
        self.ax_xy.set_xlabel('Y (m) (lateral)')
        self.ax_xy.set_ylabel('X (m) (forward, up)')

        # Per-leg phase and Z axes (right two columns)
        self.ax_phase_list = []
        self.ax_z_list = []
        for i in range(6):
            axp = fig.add_subplot(gs[i, 1])
            axp.set_ylabel(LEG_NAMES[i])
            axp.set_ylim(0.0, TWO_PI)
            axp.grid(True)
            # only label x on bottom subplot
            if i == 5:
                axp.set_xlabel('Time (s)')
            else:
                axp.set_xticklabels([])

            axz = fig.add_subplot(gs[i, 2])
            axz.grid(True)
            if i == 5:
                axz.set_xlabel('Time (s)')
            else:
                axz.set_xticklabels([])

            self.ax_phase_list.append(axp)
            self.ax_z_list.append(axz)

        self.canvas = FigureCanvasTkAgg(fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

        # Z history buffers
        self.hist_len = HIST_LEN
        self.z_hist = [deque([0.0] * self.hist_len, maxlen=self.hist_len) for i in range(6)]

        # time axis for waveform (seconds)
        self.time_axis = np.linspace(- (self.hist_len - 1) * FRAME_MS / 1000.0, 0.0, self.hist_len)

        # Phase history buffers
        self.main_phase_hist = deque([0.0] * self.hist_len, maxlen=self.hist_len)
        self.phase_hist = [deque([0.0] * self.hist_len, maxlen=self.hist_len) for _ in range(6)]

        # pre-create line artists: for each leg, plot main phase (faint) and leg phase on its own axis
        self.main_phase_lines = []
        self.phase_lines = []
        for i in range(6):
            (ln_main,) = self.ax_phase_list[i].plot(self.time_axis, list(self.main_phase_hist), color='k', linewidth=1.0, alpha=0.6)
            (ln_leg,) = self.ax_phase_list[i].plot(self.time_axis, list(self.phase_hist[i]), color=COLORS[i], label=LEG_NAMES[i])
            self.main_phase_lines.append(ln_main)
            self.phase_lines.append(ln_leg)

        # pre-create line artists for Z on each leg's axis
        self.z_lines = []
        for i in range(6):
            (ln,) = self.ax_z_list[i].plot(self.time_axis, list(self.z_hist[i]), color=COLORS[i], label=LEG_NAMES[i])
            self.z_lines.append(ln)

        self.running = False
        # time accumulator for running gait.loop at controller period DT
        self._last_time = time.perf_counter()
        self._accum_time = 0.0

    def start(self):
        if self.running:
            return
        # create gait instance from selected pattern at start
        try:
            selected = str(self.gait_var.get())
        except Exception:
            selected = 'tripod'
        self.gait = HexapodGait(gait=selected)
        # apply current parameter values to new gait
        try:
            self.gait.set_max_forward_speed(float(self.max_forward_speed_var.get()))
        except Exception:
            pass
        try:
            self.gait.set_max_swing_speed(float(self.max_swing_speed_var.get()))
        except Exception:
            pass
        try:
            self.gait.set_max_turn_rate(float(self.turn_rate_var.get()))
        except Exception:
            pass
        try:
            # apply stance_fraction from UI at start
            self.gait.set_stance_fraction(float(self.stance_var.get()))
        except Exception:
            pass

        # disable gait selector while running
        self.gait_combo.config(state='disabled')

        self.running = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self._loop()

    def stop(self):
        self.running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        # re-enable gait selector
        try:
            self.gait_combo.config(state='readonly')
        except Exception:
            pass

    def _loop(self):
        if not self.running:
            return

        # frame timing: accumulate real elapsed time and execute controller
        now = time.perf_counter()
        frame_elapsed = now - self._last_time
        self._last_time = now
        self._accum_time += frame_elapsed

        start_time = time.perf_counter()

        speed = float(self.speed_var.get())
        direction = float(self.dir_var.get())
        rotation = float(self.rot_var.get())

        # Apply adjustable rotation parameters to gait
        try:
            self.gait.set_max_turn_rate(float(self.turn_rate_var.get()))
        except Exception:
            pass
        # Apply adjustable speed parameters to gait and update per-step coefficients
        try:
            mfs = float(self.max_forward_speed_var.get())
            self.gait.set_max_forward_speed(mfs)
        except Exception:
            pass
        try:
            mss = float(self.max_swing_speed_var.get())
            self.gait.set_max_swing_speed(mss)
        except Exception:
            pass

        # Apply stance_fraction from UI
        try:
            sf = float(self.stance_var.get())
            # clamp 0..1
            sf = max(0.0, min(1.0, sf))
            self.gait.stance_fraction = sf
            self.stance_val_var.set(f"{sf:.3f}")
        except Exception:
            pass

        # Update displayed slider numeric values
        try:
            self.speed_val_var.set(f"{speed:.3f}")
            self.dir_val_var.set(f"{direction:.3f}")
            self.rot_val_var.set(f"{rotation:.3f}")
            self.turn_rate_val_var.set(f"{float(self.turn_rate_var.get()):.3f}")
            # update numeric displays for forward/swing speed
            mfs = float(self.max_forward_speed_var.get())
            mss = float(self.max_swing_speed_var.get())
            self.max_forward_speed_val_var.set(f"{mfs:.3f}")
            self.max_swing_speed_val_var.set(f"{mss:.3f}")
        except Exception:
            pass

        # Run controller enough steps to consume accumulated time (DT steps)
        outputs = None
        steps = 0
        try:
            while self._accum_time >= DT:
                outputs = self.gait.step(speed, direction, rotation)
                self._accum_time -= DT
                steps += 1
        except Exception:
            # Ensure at least one call if something goes wrong in the loop.
            outputs = self.gait.step(speed, direction, rotation)
            steps = 1

        # If no controller steps were executed (frame faster than DT), run one step
        if steps == 0:
            outputs = self.gait.step(speed, direction, rotation)

        # Update state display
        self.state_var.set(self.gait.state.name)

        self._draw(outputs)

        # Calculate remaining time to maintain consistent FRAME_MS period
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        delay_ms = max(1, int(FRAME_MS - elapsed_ms))
        # Estimate FPS from total frame period (processing + scheduled delay)
        try:
            total_ms = elapsed_ms + delay_ms
            fps = 1000.0 / total_ms if total_ms > 0.0 else 0.0
            self.fps_var.set(f"{fps:.1f}")
        except Exception:
            pass

        self.root.after(delay_ms, self._loop)

    def _draw(self, outputs):
        # Draw top-down XY
        self.ax_xy.cla()
        # draw body center
        self.ax_xy.scatter([0.0], [0.0], c='k', s=20)


        # Now plot with forward (X) mapped to vertical axis (up), lateral (Y) to horizontal axis
        # Get leg home positions (base) from gait controller
        home_positions = self.gait.leg_home_positions.targets
        
        for i in range(6):
            out = outputs.targets[i]
            home = home_positions[i]
            
            # map: plot_x = lateral (Y), plot_y = forward (X)
            # foot position from outputs (body-centered coordinates)
            foot_x = out.y  # lateral
            foot_y = out.x  # forward
            # home position as leg base
            base_x = home.y  # lateral
            base_y = home.x  # forward

            # draw leg line and foot
            # determine contact by z == 0.0
            grounded = (out.z == 0.0)
            color = 'b' if grounded else 'r'
            self.ax_xy.plot([base_x, foot_x], [base_y, foot_y], c='k', linewidth=1)
            self.ax_xy.scatter([base_x], [base_y], c='gray')
            self.ax_xy.scatter([foot_x], [foot_y], c=color, s=40)
            self.ax_xy.text(foot_x, foot_y + 0.01, LEG_NAMES[i])

            # update Z history
            self.z_hist[i].append(out.z)

        # build grounded point list (plot coords)
        grounded_pts = []
        for i in range(6):
            out = outputs.targets[i]
            grounded = (out.z == 0.0)
            if grounded:
                # foot position in plot coords (Y=lateral, X=forward)
                px = out.y  # lateral
                py = out.x  # forward
                grounded_pts.append((px, py))

        # draw convex hull of grounded feet if available
        if len(grounded_pts) >= 3:
            hull = convex_hull(grounded_pts)
            hx = [p[0] for p in hull] + [hull[0][0]]
            hy = [p[1] for p in hull] + [hull[0][1]]
            self.ax_xy.fill(hx, hy, color='gray', alpha=0.25)
        elif len(grounded_pts) == 2:
            # draw filled polygon as thin trapezoid between two points
            a, b = grounded_pts
            self.ax_xy.plot([a[0], b[0]], [a[1], b[1]], c='gray')
        elif len(grounded_pts) == 1:
            p = grounded_pts[0]
            self.ax_xy.scatter([p[0]], [p[1]], c='gray', s=50, alpha=0.3)

        # append phase history (main + per-leg) from gait's stored members
        main_phase = getattr(self.gait, 'global_phase', getattr(self.gait, 'main_phase', 0.0))
        self.main_phase_hist.append(main_phase)
        leg_phases = getattr(self.gait, 'per_leg_phases', getattr(self.gait, 'leg_phases', [0.0]*6))
        for i in range(6):
            self.phase_hist[i].append(leg_phases[i])

        # Fixed XY limits (no autoscale)
        self.ax_xy.set_xlim(-PLOT_LATERAL_RANGE, PLOT_LATERAL_RANGE)
        self.ax_xy.set_ylim(-PLOT_FORWARD_RANGE, PLOT_FORWARD_RANGE)

        # update Z lines and per-leg Z axes limits
        for i in range(6):
            zdata = list(self.z_hist[i])
            self.z_lines[i].set_ydata(zdata)
            if zdata:
                zmin, zmax = min(zdata), max(zdata)
                if zmin == zmax:
                    zmin -= 0.02; zmax += 0.02
                self.ax_z_list[i].set_ylim(zmin - 0.01, zmax + 0.01)
            self.ax_z_list[i].set_xlim(self.time_axis[0], self.time_axis[-1])

        # update phase lines and per-leg phase axes
        for i in range(6):
            self.main_phase_lines[i].set_ydata(list(self.main_phase_hist))
            self.phase_lines[i].set_ydata(list(self.phase_hist[i]))
            self.ax_phase_list[i].set_xlim(self.time_axis[0], self.time_axis[-1])
            self.ax_phase_list[i].set_ylim(0.0, TWO_PI)

        self.canvas.draw()


if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    root.protocol('WM_DELETE_WINDOW', root.quit)
    root.mainloop()
