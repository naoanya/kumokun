#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kumo-kun 3D Visualizer
Application that displays the robot's current pose in real-time in 3D
"""
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# For 3D plotting (required import)
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import os
import json
import threading
import time
import math

# Add py_utils to sys.path so local imports work
current_dir = os.path.dirname(os.path.abspath(__file__))
py_utils_path = os.path.join(os.path.dirname(current_dir), 'py_utils')
if py_utils_path not in sys.path:
    sys.path.append(py_utils_path)

from kumokun_controller import KumokunController
from servo_converter import ServoConverter
from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG


class RobotClient:
    """Manages robot communication and the kinematics model"""
    def __init__(self):
        # Use the high-level stateful controller which provides a KumokunServo
        # wrapper at `controller.controller`.
        self.kumokun = KumokunController()
        self.controller = self.kumokun.controller
        self.kinematics = KumokunKinematics()
        self.servo_converters = self._create_servo_converters()

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

    @property
    def is_connected(self):
        return self.controller.is_connected

    def list_ports(self):
        return self.controller.list_ports()

    def connect(self, port, use_dummy: bool = False):
        # Recreate KumokunController with requested dummy flag and connect
        try:
            self.kumokun = KumokunController(use_dummy=use_dummy)
            self.controller = self.kumokun.controller
        except Exception:
            # Fallback: attempt to keep existing controller
            pass

        # Set requested port on controller and connect; start controller thread
        self.kumokun.port = port
        success, msg = self.kumokun.connect()
        if success:
            try:
                self.kumokun.start()
            except Exception:
                pass
        return success, msg

    def disconnect(self):
        if self.is_connected:
            try:
                self.kumokun.change_state(self.kumokun.current_state)
            except Exception:
                pass
            self.kumokun.disconnect()

    def update_pose(self):
        """Read current positions from servos and update the kinematics model"""
        if not self.is_connected:
            return False

        # Request a kinematics snapshot from the controller (fresh FK run inside controller)
        try:
            ks = self.kumokun.get_kinematics_state()
        except Exception:
            return False

        if not ks:
            return False

        try:
            # ks is now a KumokunKinematics instance populated by the controller
            if not isinstance(ks, KumokunKinematics):
                return False

            # Replace local kinematics with the controller-provided instance.
            # Assigning the instance directly keeps absolute positions and angles
            # already computed by get_kinematics_state().
            try:
                self.kinematics = ks
            except Exception:
                return False

            return True
        except Exception:
            return False

    def send_pose_from_kinematics(self):
        """Send the current angles from the kinematics model to the robot"""
        if not self.is_connected:
            return

        # Build a position list ordered by physical servo id (1..18)
        positions = [7500] * 18
        for i in range(6):
            leg = self.kinematics.get_leg(i)
            base_sid = i * 3
            # Software sids: Link3=base+0, Link2=base+1, Link1=base+2
            for sw_sid, angle in ((base_sid + 0, leg.link3_servo.ik_angle_deg),
                                  (base_sid + 1, leg.link2_servo.ik_angle_deg),
                                  (base_sid + 2, leg.link1_servo.ik_angle_deg)):
                if sw_sid not in SERVO_CONFIG:
                    continue
                conf = SERVO_CONFIG[sw_sid]
                converter = self.servo_converters[sw_sid]
                try:
                    val = converter.convert_to_value(angle)
                except Exception as e:
                    print(f"Servo conversion error for sid {sw_sid}: {e}")
                    continue
                phys = conf["physical_sid"]
                if 1 <= phys <= 18:
                    positions[phys - 1] = val

        # Send in bulk
        ok, msg = self.controller.set_all_pos(positions)
        if not ok:
            print(f"Failed to send positions: {msg}")

    def _send_servo_angle(self, servo_sid, angle):
        if servo_sid not in SERVO_CONFIG:
            return
        conf = SERVO_CONFIG[servo_sid]
        converter = self.servo_converters[servo_sid]
        try:
            val = converter.convert_to_value(angle)
        except Exception as e:
            print(f"Servo conversion error for sid {servo_sid}: {e}")
            return
        # Use wrapper API set_pos (physical id)
        ok, msg = self.controller.set_pos(conf["physical_sid"], val)
        if not ok:
            print(f"Failed to set pos for {conf['physical_sid']}: {msg}")


class VisualizerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Kumo-kun 3D Visualizer")
        self.root.geometry("1000x800")
        
        self.robot = RobotClient()
        self.auto_update = False
        self._plot_update_pending = False  # Flag to prevent event queue overflow
        
        self.setup_ui()
        self.update_port_list()
        
        # Initial drawing
        self.update_plot()
        
        # Bind event used to request plot updates from threads
        self.root.bind("<<UpdatePlot>>", self._handle_update_plot)
        
        # Set window close handler
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        # --- Left control panel ---
        self.control_frame = ttk.Frame(self.root, padding=10)
        self.control_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        self._setup_connection_group(self.control_frame)
        self._setup_operation_group(self.control_frame)
        self._setup_angle_display(self.control_frame)
        self._setup_status_bar(self.control_frame)

        # --- Right 3D display area ---
        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self._setup_plot_area(self.plot_frame)

    def _setup_connection_group(self, parent):
        # Connection settings group
        conn_group = ttk.LabelFrame(parent, text="Connection", padding=10)
        conn_group.pack(fill=tk.X, pady=5)
        
        ttk.Label(conn_group, text="Port:").pack(anchor=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_group, textvariable=self.port_var, state='readonly')
        self.port_combo.pack(fill=tk.X, pady=2)
        
        ttk.Button(conn_group, text="Refresh Ports", command=self.update_port_list).pack(fill=tk.X, pady=2)
        # Dummy controller toggle
        self.use_dummy_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(conn_group, text="Use Dummy (simulate)", variable=self.use_dummy_var).pack(fill=tk.X, pady=2)
        
        self.btn_connect = ttk.Button(conn_group, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(fill=tk.X, pady=5)

    def _setup_operation_group(self, parent):
        # Operation group
        op_group = ttk.LabelFrame(parent, text="Operations", padding=10)
        op_group.pack(fill=tk.X, pady=5)
        
        self.btn_read = ttk.Button(op_group, text="Read Pose (Single)", command=self.read_pose_single, state=tk.DISABLED)
        self.btn_read.pack(fill=tk.X, pady=2)
        
        self.chk_auto_var = tk.BooleanVar(value=False)
        self.chk_auto = ttk.Checkbutton(op_group, text="Auto Update (Realtime)", variable=self.chk_auto_var, command=self.toggle_auto_update, state=tk.DISABLED)
        self.chk_auto.pack(fill=tk.X, pady=5)

        # State control buttons
        states = [
            "HOMING", "STANDUP", "WALK_MODE0", "WALK_MODE1",
            "STANDDOWN", "POWERDOWN", "POWERUP", "IDLE"
        ]
        self.state_buttons = {}
        state_frame = ttk.LabelFrame(op_group, text="Change State", padding=5)
        state_frame.pack(fill=tk.X, pady=5)
        for s in states:
            btn = ttk.Button(state_frame, text=s.replace("_", " "), command=lambda ss=s: self._on_state_button(ss), state=tk.DISABLED)
            btn.pack(fill=tk.X, pady=2)
            self.state_buttons[s] = btn

    def _setup_motion_group(self, parent):
        # MotionController UI removed — motion handled externally or offline.
        return

    def _setup_angle_display(self, parent):
        # Servo angle display panel
        angle_group = ttk.LabelFrame(parent, text="Servo Angles (Deg)", padding=5)
        angle_group.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Header
        headers = ["Leg", "Knee", "HipY", "HipZ"]
        for col, text in enumerate(headers):
            ttk.Label(angle_group, text=text, font=("", 8, "bold")).grid(row=0, column=col, padx=2, pady=2)
            
        self.angle_labels = {}
        for i in range(6):
            ttk.Label(angle_group, text=f"L{i}", font=("", 8)).grid(row=i+1, column=0, padx=2, pady=1)
            
            # Knee
            lbl_knee = ttk.Label(angle_group, text="0.0", width=6, anchor="e", font=("", 8))
            lbl_knee.grid(row=i+1, column=1, padx=2, pady=1)
            self.angle_labels[f"L{i}_Knee"] = lbl_knee
            
            # HipY
            lbl_hipy = ttk.Label(angle_group, text="0.0", width=6, anchor="e", font=("", 8))
            lbl_hipy.grid(row=i+1, column=2, padx=2, pady=1)
            self.angle_labels[f"L{i}_HipY"] = lbl_hipy
            
            # HipZ
            lbl_hipz = ttk.Label(angle_group, text="0.0", width=6, anchor="e", font=("", 8))
            lbl_hipz.grid(row=i+1, column=3, padx=2, pady=1)
            self.angle_labels[f"L{i}_HipZ"] = lbl_hipz

    def _setup_status_bar(self, parent):
        # Status display
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(parent, textvariable=self.status_var, wraplength=180, foreground="blue").pack(side=tk.BOTTOM, pady=10)

    def _setup_plot_area(self, parent):
        # Create Matplotlib Figure
        self.fig = plt.Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Embed into Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def update_port_list(self):
        ports = self.robot.list_ports()
        self.port_combo['values'] = ports
        if ports and ports[0] != "No ports available":
            self.port_combo.current(0)

    def toggle_connection(self):
        if not self.robot.is_connected:
            # Allow empty port when using dummy controller
            port = self.port_var.get().strip()
            use_dummy = False
            try:
                use_dummy = bool(self.use_dummy_var.get())
            except Exception:
                use_dummy = False

            if not port and not use_dummy:
                return

            print(f"Use Dummy = {use_dummy}")
            success, msg = self.robot.connect(port or None, use_dummy=use_dummy)
            if success:
                self.btn_connect.config(text="Disconnect")
                self.btn_read.config(state=tk.NORMAL)
                self.chk_auto.config(state=tk.NORMAL)
                for b in self.state_buttons.values():
                    try:
                        b.config(state=tk.NORMAL)
                    except Exception:
                        pass
                # Motion controls removed
                self.status_var.set("Connected")
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            self.auto_update = False
            self.chk_auto_var.set(False)
            self.robot.disconnect()
            self.btn_connect.config(text="Connect")
            self.btn_read.config(state=tk.DISABLED)
            self.chk_auto.config(state=tk.DISABLED)
            for b in self.state_buttons.values():
                try:
                    b.config(state=tk.DISABLED)
                except Exception:
                    pass
            # Motion controls removed
            self.status_var.set("Disconnected")
            
    def on_closing(self):
        """Cleanup when the window is closed"""
        # Stop auto update first
        self.auto_update = False
        # Uncheck the checkbox
        try:
            self.chk_auto_var.set(False)
        except Exception:
            pass
        # Give thread time to exit
        time.sleep(0.1)
        # MotionController removed
        if self.robot.is_connected:
            self.robot.disconnect()
        self.root.destroy()

    def read_pose_single(self):
        if not self.robot.is_connected:
            return
        self.status_var.set("Reading servos...")
        # Run in a separate daemon thread to avoid freezing the UI and to
        # allow the application to exit even if the thread is running.
        t = threading.Thread(target=self._read_pose_task)
        t.daemon = True
        t.start()

    def toggle_auto_update(self):
        if self.chk_auto_var.get():
            # Ideally should prevent reading while motion is active,
            # but here we keep it simple and leave it to the user
            # (Auto Update should be OFF during motion).
            self.auto_update = True
            self.status_var.set("Auto updating...")
            t = threading.Thread(target=self._auto_update_task, daemon=True)
            t.start()
        else:
            self.auto_update = False
            self.status_var.set("Auto update stopped")

    def set_motion(self, command):
        # MotionController removed; this action is a no-op
        self.status_var.set(f"Motion: {command} (disabled)")

    def update_motion_params(self, _=None):
        # MotionController removed; parameters are not applied
        return

    def trigger_plot_update(self):
        """Safely request a plot update from a thread"""
        # Only generate event if one isn't already pending to prevent queue overflow
        if not self._plot_update_pending:
            self._plot_update_pending = True
            try:
                self.root.event_generate("<<UpdatePlot>>", when="tail")
            except Exception:
                self._plot_update_pending = False

    def _auto_update_task(self):
        while self.auto_update and self.robot.is_connected:
            try:
                # Update the robot state
                self.robot.update_pose()
                # Request plot update
                self.trigger_plot_update()
            except Exception as e:
                print(f"Error in auto update: {e}")
            
            # Sleep in small chunks to allow quicker exit when auto_update becomes False
            for _ in range(5):  # 5 * 0.01 = 0.05 seconds total
                if not self.auto_update:
                    break
                time.sleep(0.01)

    def _read_pose_task(self):
        # Update the robot state
        self.robot.update_pose()
        
        self.trigger_plot_update()

    def _on_state_button(self, state_name: str):
        """Handle state-change button presses."""
        try:
            # Use controller's process_command to validate and change state
            kumokun = getattr(self.robot, 'kumokun', None)
            if kumokun is None:
                messagebox.showerror("Error", "Controller not available")
                return
            success, msg = kumokun.process_command(state_name)
            if success:
                self.status_var.set(msg)
            else:
                messagebox.showerror("State Change Failed", msg)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _handle_update_plot(self, event):
        try:
            self.update_plot()
            self.update_angle_labels()
            self.status_var.set("Updated")
        except Exception as e:
            print(f"Error updating plot: {e}")
            self.status_var.set("Update error")
        finally:
            # Reset the pending flag to allow next update
            self._plot_update_pending = False

    def update_angle_labels(self):
        for i in range(6):
            leg = self.robot.kinematics.get_leg(i)
            if leg:
                self.angle_labels[f"L{i}_Knee"].config(text=f"{leg.link3_servo.ik_angle_deg:.1f}")
                self.angle_labels[f"L{i}_HipY"].config(text=f"{leg.link2_servo.ik_angle_deg:.1f}")
                self.angle_labels[f"L{i}_HipZ"].config(text=f"{leg.link1_servo.ik_angle_deg:.1f}")

    def update_plot(self):
        # Save current view angles
        current_azim = getattr(self.ax, 'azim', None)
        current_elev = getattr(self.ax, 'elev', None)

        self.ax.clear()
        
        # Axis limits (adjusted to robot size)
        limit = 250
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(-150, 150)
        self.ax.set_xlabel('X [mm]')
        self.ax.set_ylabel('Y [mm]')
        self.ax.set_zlabel('Z [mm]')
        self.ax.set_title("Robot Pose")
        
        # Body center (effective origin = body_center_pos + default height)
        body_origin = self.robot.kinematics.body_center_pos + np.array([0.0, 0.0, self.robot.kinematics.default_body_height])
        
        # Draw each leg
        colors = ['r', 'g', 'b', 'c', 'm', 'y']
        
        for i in range(6):
            leg = self.robot.kinematics.get_leg(i)
            
            # Get absolute coordinates for each joint (map to current kinematics API)
            p_origin = body_origin
            p_hip_z = leg.link1_servo.absolute_position
            p_hip_y = leg.link2_servo.absolute_position
            p_knee = leg.link3_servo.absolute_position
            p_toe = leg.end_effector.absolute_position
            
            # Link connections: Origin -> HipZ -> HipY -> Knee -> Toe
            xs = [p_origin[0], p_hip_z[0], p_hip_y[0], p_knee[0], p_toe[0]]
            ys = [p_origin[1], p_hip_z[1], p_hip_y[1], p_knee[1], p_toe[1]]
            zs = [p_origin[2], p_hip_z[2], p_hip_y[2], p_knee[2], p_toe[2]]
            
            self.ax.plot(xs, ys, zs, marker='o', color=colors[i], label=f'Leg {i}')

        # Restore view angles
        if current_azim is not None and current_elev is not None:
            self.ax.view_init(elev=current_elev, azim=current_azim)

        self.canvas.draw()


# MotionController removed — gait generation is intentionally out of GUI.


if __name__ == "__main__":
    root = tk.Tk()
    app = VisualizerApp(root)
    root.mainloop()