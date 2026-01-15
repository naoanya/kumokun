#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kumo-kun Workspace Visualizer
Tool to visualize leg reachable workspace using kumokun_kinematics.py
"""
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import sys
import os

# Add py_utils to sys.path for local imports
current_dir = os.path.dirname(os.path.abspath(__file__))
py_utils_path = os.path.join(os.path.dirname(current_dir), 'py_utils')
if py_utils_path not in sys.path:
    sys.path.append(py_utils_path)

from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG
from servo_converter import ServoConverter, ServoConversionError

# Constants for visualization and grid
# Axis limits for each subplot (absolute value in mm)
PLOT_AXIS_LIMIT = 350
# Z heights (start, end inclusive, step) in mm
Z_HEIGHT_START = -350
Z_HEIGHT_END = 200
Z_HEIGHT_STEP = 25
# Derived array of Z heights
Z_HEIGHTS = np.arange(Z_HEIGHT_START, Z_HEIGHT_END + 1, Z_HEIGHT_STEP)
# Grid layout
NUM_COLS = 8
# Default sampling resolution (mm)
RESOLUTION_DEFAULT = 15

class WorkspaceVisualizerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Kumo-kun Workspace Visualizer")
        self.root.geometry("1400x900")
        
        self.kinematics = KumokunKinematics()
        self.leg_id = 0 # Fixed to Leg 0
        self.servo_converters = self._create_servo_converters()
        
        # Z heights (from constants)
        self.z_heights = Z_HEIGHTS
        
        self._closing = False
        self.setup_ui()
        # Set WM_DELETE_WINDOW handler
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        # Calculate once for initial display
        self.root.after(100, self.plot_workspace)

    def setup_ui(self):
        # Control Panel
        control_frame = ttk.Frame(self.root, padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        ttk.Label(control_frame, text="Settings (Leg 0)", font=("", 12, "bold")).pack(pady=10)
        
        # Resolution
        ttk.Label(control_frame, text="Resolution (Step mm):").pack(anchor=tk.W, pady=(10, 0))
        self.res_var = tk.IntVar(value=RESOLUTION_DEFAULT)
        ttk.Spinbox(control_frame, from_=5, to=50, textvariable=self.res_var, width=5).pack(anchor=tk.W, pady=5)
        
        # Calculate Button
        self.calc_button = ttk.Button(control_frame, text="Recalculate Workspace", command=self.plot_workspace)
        self.calc_button.pack(fill=tk.X, pady=20)
        
        # Info
        self.info_label = ttk.Label(control_frame, text="Ready", wraplength=180)
        self.info_label.pack(side=tk.BOTTOM, pady=10)

        # Plot Area - Grid of subplots
        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create grid of subplots using constants
        num_cols = NUM_COLS
        num_rows = int(np.ceil(len(self.z_heights) / num_cols))
        self.fig, self.axes = plt.subplots(num_rows, num_cols, figsize=(18, 4 * num_rows), dpi=80)
        self.fig.suptitle("Leg 0 Workspace at Different Z Heights", fontsize=14, fontweight='bold')
        self.fig.tight_layout(rect=[0, 0, 1, 0.98], h_pad=2, w_pad=2)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def _create_servo_converters(self):
        """Create ServoConverter instances for all configured servos."""
        converters = {}
        for sid, conf in SERVO_CONFIG.items():
            converters[sid] = ServoConverter(
                direction=conf["direction"],
                offset=conf["offset"],
                min_angle=conf.get("min_angle"),
                max_angle=conf.get("max_angle"),
            )
        expected_sids = set(range(6 * 3))
        missing = sorted(expected_sids - set(converters.keys()))
        if missing:
            raise ValueError(f"Missing SERVO_CONFIG for sid(s): {missing}")
        return converters

    def plot_workspace(self):
        self.info_label.config(text="Calculating...\nPlease wait.")
        self.root.update()
        
        step = self.res_var.get()
        leg_id = self.leg_id
        
        # Define search range (mm) using the axis limit constant
        x_range = np.arange(-PLOT_AXIS_LIMIT, PLOT_AXIS_LIMIT + 1, step)
        y_range = np.arange(-PLOT_AXIS_LIMIT, PLOT_AXIS_LIMIT + 1, step)
        
        # Get leg info for mounting angle visualization
        leg = self.kinematics.get_leg(leg_id)
        mount_angle = leg.mount_angle_deg
        
        # Flatten axes array for easier indexing
        axes_flat = self.axes.flatten()
        
        total_valid = 0
        total_limit = 0
        
        # Disable calculate button while running
        try:
            self.calc_button.config(state=tk.DISABLED)
        except Exception:
            pass

        # Plot for each Z height
        for idx, z_height in enumerate(self.z_heights):
            if idx >= len(axes_flat):
                break
                
            ax = axes_flat[idx]
            ax.clear()
            
            valid_x = []
            valid_y = []
            limit_x = []
            limit_y = []
            
            # Calculate workspace at this Z height
            for x in x_range:
                    # Check closing flag to allow graceful exit
                if getattr(self, '_closing', False):
                    self.info_label.config(text="Aborted")
                    # re-enable button and return
                    try:
                        self.calc_button.config(state=tk.NORMAL)
                    except Exception:
                        pass
                    return
                for y in y_range:
                    target_pos = np.array([float(x), float(y), float(z_height)])
                    
                    # Use do_fk=False for speed
                    ret = self.kinematics.solve_ik_for_leg(leg_id, target_pos, do_fk=False)
                    
                    if ret == 0:
                        # Check servo limits via ServoConverter
                        is_valid = True
                        base_sid = leg_id * 3
                        sid_link3 = base_sid + 0
                        sid_link2 = base_sid + 1
                        sid_link1 = base_sid + 2

                        try:
                            self.servo_converters[sid_link3].convert_to_value(leg.link3_servo.ik_angle_deg)
                            self.servo_converters[sid_link2].convert_to_value(leg.link2_servo.ik_angle_deg)
                            self.servo_converters[sid_link1].convert_to_value(leg.link1_servo.ik_angle_deg)
                        except ServoConversionError:
                            is_valid = False
                        
                        if is_valid:
                            valid_x.append(x)
                            valid_y.append(y)
                        else:
                            limit_x.append(x)
                            limit_y.append(y)
            
            total_valid += len(valid_x)
            total_limit += len(limit_x)
            
            # Plot results on this subplot
            ax.set_title(f"Z={z_height:.0f}mm", fontsize=9)
            ax.set_xlabel("X [mm]", fontsize=7)
            ax.set_ylabel("Y [mm]", fontsize=7)
            ax.set_aspect('equal')
            ax.grid(True, linestyle='--', alpha=0.4)
            ax.tick_params(labelsize=6)
            # Fix axis limits for consistent scale
            ax.set_xlim(-PLOT_AXIS_LIMIT, PLOT_AXIS_LIMIT)
            ax.set_ylim(-PLOT_AXIS_LIMIT, PLOT_AXIS_LIMIT)
            
            # Draw robot body center
            ax.plot(0, 0, 'k+', markersize=6)
            
            # Draw reachable and limit-exceeded points
            if valid_x:
                ax.scatter(valid_x, valid_y, c='g', s=3, alpha=0.5)
            if limit_x:
                ax.scatter(limit_x, limit_y, c='r', s=3, alpha=0.5)
            
            if not valid_x and not limit_x:
                ax.text(0, 0, "No points", ha='center', fontsize=7)
            
            # Draw leg mounting direction
            rad = np.radians(mount_angle)
            arrow_len = 80
            ax.arrow(0, 0, arrow_len * np.cos(rad), arrow_len * np.sin(rad), 
                     head_width=8, head_length=8, fc='b', ec='b', linewidth=0.8)
        
        # Hide unused subplots
        for idx in range(len(self.z_heights), len(axes_flat)):
            axes_flat[idx].set_visible(False)
        
        self.canvas.draw()
        
        self.info_label.config(text=f"Complete!\nTotal valid: {total_valid}\nLimit exceeded: {total_limit}")
        try:
            self.calc_button.config(state=tk.NORMAL)
        except Exception:
            pass

    def on_closing(self):
        # Signal running plot to stop
        self._closing = True
        # Close matplotlib figure to free resources
        try:
            plt.close(self.fig)
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

if __name__ == "__main__":
    root = tk.Tk()
    app = WorkspaceVisualizerApp(root)
    root.mainloop()