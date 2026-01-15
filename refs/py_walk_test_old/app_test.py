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

from hexapod_gait import HexapodGait, TWO_PI

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

FRAME_MS = 20  # update interval for GUI in ms
HISTORY_MS = 4000  # time window for Z waveform (ms)
HIST_LEN = max(10, HISTORY_MS // FRAME_MS)

# Fixed 2D plot size (half-range in meters)
PLOT_LATERAL_RANGE = 0.3  # lateral half-range (Y axis, horizontal)
PLOT_FORWARD_RANGE = 0.3  # forward half-range (X axis, vertical)

# Define example leg mount positions (body-relative) for visualization
# Place six legs on a circle so each foot base is equidistant from the center.
R = 0.21  # radius (m)
# Angles chosen to place LF,LM,LR on the left half and RF,RM,RR on the right half
ANGLES_DEG = [-30, -90, -150, 30, 90, 150]
LEG_BASES = [
    np.array([R * math.cos(math.radians(a)), R * math.sin(math.radians(a)), -0.08])
    for a in ANGLES_DEG
]

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

        ttk.Label(ctrl, text='Direction (rad)').pack()
        self.dir_var = tk.DoubleVar(value=0.0)
        ttk.Scale(ctrl, from_=-math.pi, to=math.pi, orient=tk.HORIZONTAL, variable=self.dir_var).pack()

        ttk.Label(ctrl, text='Rotation').pack()
        self.rot_var = tk.DoubleVar(value=0.0)
        ttk.Scale(ctrl, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, variable=self.rot_var).pack()

        # Turn parameter controls
        ttk.Label(ctrl, text='Max Turn Rate (rad/s)').pack(pady=(8,0))
        self.turn_rate_var = tk.DoubleVar(value=getattr(self.gait, 'max_turn_rate', 1.0))
        ttk.Scale(ctrl, from_=0.0, to=3.0, orient=tk.HORIZONTAL, variable=self.turn_rate_var).pack()

        ttk.Label(ctrl, text='Turn Scale (m)').pack(pady=(8,0))
        self.turn_scale_var = tk.DoubleVar(value=getattr(self.gait, 'turn_scale', 0.02))
        ttk.Scale(ctrl, from_=0.0, to=0.1, orient=tk.HORIZONTAL, variable=self.turn_scale_var).pack()

        self.start_btn = ttk.Button(ctrl, text='Start', command=self.start)
        self.start_btn.pack(fill=tk.X, pady=(10,2))
        self.stop_btn = ttk.Button(ctrl, text='Stop', command=self.stop, state=tk.DISABLED)
        self.stop_btn.pack(fill=tk.X)

        # Canvas / Matplotlib with three subplots stacked vertically:
        fig = Figure(figsize=(8,10))
        self.ax_xy = fig.add_subplot(311)
        self.ax_z = fig.add_subplot(312)
        self.ax_phase = fig.add_subplot(313)

        self.ax_xy.set_aspect('equal', adjustable='box')
        self.ax_xy.set_xlabel('Y (m) (lateral)')
        self.ax_xy.set_ylabel('X (m) (forward, up)')

        self.ax_z.set_xlabel('Time (s)')
        self.ax_z.set_ylabel('Z (m)')
        self.ax_z.grid(True)
        self.ax_phase.set_xlabel('Time (s)')
        self.ax_phase.set_ylabel('Phase (rad)')
        self.ax_phase.grid(True)

        self.canvas = FigureCanvasTkAgg(fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

        # Z history buffers
        self.hist_len = HIST_LEN
        self.z_hist = [deque([LEG_BASES[i][2]] * self.hist_len, maxlen=self.hist_len) for i in range(6)]

        # time axis for waveform (seconds)
        self.time_axis = np.linspace(- (self.hist_len - 1) * FRAME_MS / 1000.0, 0.0, self.hist_len)

        # pre-create line artists for Z
        self.z_lines = []
        for i in range(6):
            (ln,) = self.ax_z.plot(self.time_axis, list(self.z_hist[i]), color=COLORS[i], label=LEG_NAMES[i])
            self.z_lines.append(ln)
        self.ax_z.legend(loc='upper right')

        # Phase history buffers
        self.main_phase_hist = deque([0.0] * self.hist_len, maxlen=self.hist_len)
        self.phase_hist = [deque([0.0] * self.hist_len, maxlen=self.hist_len) for _ in range(6)]

        # pre-create lines for phase plot (main + legs)
        (self.main_phase_line,) = self.ax_phase.plot(self.time_axis, list(self.main_phase_hist), color='k', linewidth=1.5, label='main')
        self.phase_lines = []
        for i in range(6):
            (ln,) = self.ax_phase.plot(self.time_axis, list(self.phase_hist[i]), color=COLORS[i], label=LEG_NAMES[i])
            self.phase_lines.append(ln)
        self.ax_phase.legend(loc='upper right', ncol=3, fontsize='small')

        self.running = False

    def start(self):
        if self.running:
            return
        self.running = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self._loop()

    def stop(self):
        self.running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def _loop(self):
        if not self.running:
            return

        speed = float(self.speed_var.get())
        direction = float(self.dir_var.get())
        rotation = float(self.rot_var.get())

        # Apply adjustable rotation parameters to gait
        try:
            self.gait.max_turn_rate = float(self.turn_rate_var.get())
            self.gait.turn_scale = float(self.turn_scale_var.get())
        except Exception:
            pass

        outputs = self.gait.loop(speed, direction, rotation)

        self._draw(outputs)

        self.root.after(FRAME_MS, self._loop)

    def _draw(self, outputs):
        # Draw top-down XY
        self.ax_xy.cla()
        # draw body center
        self.ax_xy.scatter([0.0], [0.0], c='k', s=20)


        # Now plot with forward (X) mapped to vertical axis (up), lateral (Y) to horizontal axis
        xs = []  # lateral (Y)
        ys = []  # forward (X)
        for i in range(6):
            base = LEG_BASES[i]
            out = outputs.get(i, {'x':0.0, 'y':0.0, 'z':base[2], 'phase':0.0})
            foot = base + np.array([out['x'], out['y'], out['z']])

            # map: plot_x = lateral (Y), plot_y = forward (X)
            base_x = base[1]
            base_y = base[0]
            foot_x = foot[1]
            foot_y = foot[0]

            # draw leg line and foot
            # determine contact by z == 0.0 per request
            grounded = (out.get('z', base[2]) == 0.0)
            color = 'b' if grounded else 'r'
            self.ax_xy.plot([base_x, foot_x], [base_y, foot_y], c='k', linewidth=1)
            self.ax_xy.scatter([base_x], [base_y], c='gray')
            self.ax_xy.scatter([foot_x], [foot_y], c=color, s=40)
            self.ax_xy.text(foot_x, foot_y + 0.01, LEG_NAMES[i])

            xs.append(foot_x)
            ys.append(foot_y)

            # update Z history
            self.z_hist[i].append(foot[2])

        # build grounded point list (plot coords)
        grounded_pts = []
        for i in range(6):
            out = outputs.get(i, {'x':0.0, 'y':0.0, 'z':LEG_BASES[i][2], 'phase':0.0})
            grounded = (out.get('z', LEG_BASES[i][2]) == 0.0)
            if grounded:
                # compute plotted foot coordinates same as above mapping
                base = LEG_BASES[i]
                foot = base + np.array([out['x'], out['y'], out['z']])
                px = foot[1]
                py = foot[0]
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
        main_phase = getattr(self.gait, 'current_main_phase', self.gait.main_phase.phase)
        self.main_phase_hist.append(main_phase)
        leg_phases = getattr(self.gait, 'leg_phases', None)
        if leg_phases is not None:
            for i in range(6):
                self.phase_hist[i].append(leg_phases[i])
        else:
            for i in range(6):
                phase_i = self.gait.leg_phase.get_phase(i, main_phase, rotation)
                self.phase_hist[i].append(phase_i)

        # Fixed XY limits (no autoscale)
        self.ax_xy.set_xlim(-PLOT_LATERAL_RANGE, PLOT_LATERAL_RANGE)
        self.ax_xy.set_ylim(-PLOT_FORWARD_RANGE, PLOT_FORWARD_RANGE)

        # update Z lines
        for i in range(6):
            self.z_lines[i].set_ydata(list(self.z_hist[i]))

        # update phase lines
        self.main_phase_line.set_ydata(list(self.main_phase_hist))
        for i in range(6):
            self.phase_lines[i].set_ydata(list(self.phase_hist[i]))

        # adjust phase y-limits
        self.ax_phase.set_ylim(0.0, TWO_PI)

        # adjust Z y-limits based on data
        all_z = [v for h in self.z_hist for v in h]
        if all_z:
            zmin, zmax = min(all_z), max(all_z)
            if zmin == zmax:
                zmin -= 0.02; zmax += 0.02
            self.ax_z.set_ylim(zmin - 0.01, zmax + 0.01)

        self.ax_z.set_xlim(self.time_axis[0], self.time_axis[-1])
        self.ax_phase.set_xlim(self.time_axis[0], self.time_axis[-1])
        self.canvas.draw()


if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    root.protocol('WM_DELETE_WINDOW', root.quit)
    root.mainloop()
