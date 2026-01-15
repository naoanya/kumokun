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

from servo_controller import ServoController
from servo_converter import ServoConverter
from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG


class RobotClient:
    """Manages robot communication and the kinematics model"""
    def __init__(self):
        self.controller = ServoController()
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

    def connect(self, port):
        return self.controller.connect(port)

    def disconnect(self):
        if self.is_connected:
            self.controller.disconnect()

    def update_pose(self):
        """Read current positions from servos and update the kinematics model"""
        if not self.is_connected:
            return False

        updated = False
        for sid, conf in SERVO_CONFIG.items():
            if not self.is_connected:
                break

            servo_id = conf["physical_sid"]
            response, err = self.controller.free(servo_id)
            
            if err:
                if not self.is_connected: break
                # print(f"ID {servo_id} Error: {err}")
                continue
            
            if not response:
                continue
            
            try:
                data = json.loads(response)
                val = data.get("pos", data.get("feedback", 0))
                
                if val < 3500 or val > 11500:
                    continue
                
                converter = self.servo_converters[sid]
                angle = converter.convert_to_degrees(val)

                leg_id = sid // 3
                mod_id = sid % 3
                
                leg = self.kinematics.get_leg(leg_id)
                if mod_id == 0: leg.link3_servo.ik_angle_deg = angle  # Knee = Link3
                elif mod_id == 1: leg.link2_servo.ik_angle_deg = angle  # HipY = Link2
                elif mod_id == 2: leg.link1_servo.ik_angle_deg = angle  # HipZ = Link1
                
                updated = True
            except Exception as e:
                print(f"Parse Error ID {servo_id}: {e}")
        
        if updated:
            self.kinematics.do_fk()
            
        return updated

    def send_pose_from_kinematics(self):
        """Send the current angles from the kinematics model to the robot"""
        if not self.is_connected:
            return

        for i in range(6):
            leg = self.kinematics.get_leg(i)
            base_sid = i * 3
            # Software sids: Link3=base+0, Link2=base+1, Link1=base+2
            self._send_servo_angle(base_sid + 0, leg.link3_servo.ik_angle_deg)
            self._send_servo_angle(base_sid + 1, leg.link2_servo.ik_angle_deg)
            self._send_servo_angle(base_sid + 2, leg.link1_servo.ik_angle_deg)

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
        self.controller.set_pos_light(conf["physical_sid"], val)


class VisualizerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Kumo-kun 3D Visualizer")
        self.root.geometry("1000x800")
        
        self.robot = RobotClient()
        self.motion_controller = MotionController(self.robot, self.trigger_plot_update)
        self.auto_update = False
        
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
        self._setup_motion_group(self.control_frame)
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

    def _setup_motion_group(self, parent):
        # Motion control group
        motion_group = ttk.LabelFrame(parent, text="Motion Control", padding=10)
        motion_group.pack(fill=tk.X, pady=5)

        btn_frame = ttk.Frame(motion_group)
        btn_frame.pack(fill=tk.X)

        # Grid layout for controls
        #   [ Fwd ]
        # [L] [Stp] [R]
        #   [ Bwd ]
        
        self.btn_fwd = ttk.Button(btn_frame, text="Forward", command=lambda: self.set_motion("forward"), state=tk.DISABLED)
        self.btn_fwd.grid(row=0, column=1, padx=2, pady=2)
        self.btn_left = ttk.Button(btn_frame, text="Left", command=lambda: self.set_motion("turn_left"), state=tk.DISABLED)
        self.btn_left.grid(row=1, column=0, padx=2, pady=2)
        self.btn_stop = ttk.Button(btn_frame, text="Stop", command=lambda: self.set_motion("stop"), state=tk.DISABLED)
        self.btn_stop.grid(row=1, column=1, padx=2, pady=2)
        self.btn_right = ttk.Button(btn_frame, text="Right", command=lambda: self.set_motion("turn_right"), state=tk.DISABLED)
        self.btn_right.grid(row=1, column=2, padx=2, pady=2)
        self.btn_back = ttk.Button(btn_frame, text="Back", command=lambda: self.set_motion("backward"), state=tk.DISABLED)
        self.btn_back.grid(row=2, column=1, padx=2, pady=2)

        # Parameters
        param_frame = ttk.LabelFrame(motion_group, text="Parameters", padding=5)
        param_frame.pack(fill=tk.X, pady=5)

        ttk.Label(param_frame, text="Stride [mm]:").pack(anchor=tk.W)
        self.stride_var = tk.DoubleVar(value=self.motion_controller.stride)
        ttk.Scale(param_frame, from_=10.0, to=100.0, variable=self.stride_var, command=self.update_motion_params).pack(fill=tk.X)

        ttk.Label(param_frame, text="Step Height [mm]:").pack(anchor=tk.W)
        self.height_var = tk.DoubleVar(value=self.motion_controller.step_height)
        ttk.Scale(param_frame, from_=10.0, to=80.0, variable=self.height_var, command=self.update_motion_params).pack(fill=tk.X)

        ttk.Label(param_frame, text="Cycle Time [s]:").pack(anchor=tk.W)
        self.cycle_var = tk.DoubleVar(value=self.motion_controller.cycle_time)
        ttk.Scale(param_frame, from_=0.2, to=2.0, variable=self.cycle_var, command=self.update_motion_params).pack(fill=tk.X)

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
            port = self.port_var.get()
            if not port:
                return
            
            success, msg = self.robot.connect(port)
            if success:
                self.btn_connect.config(text="Disconnect")
                self.btn_read.config(state=tk.NORMAL)
                self.chk_auto.config(state=tk.NORMAL)
                self.btn_fwd.config(state=tk.NORMAL)
                self.btn_back.config(state=tk.NORMAL)
                self.btn_left.config(state=tk.NORMAL)
                self.btn_right.config(state=tk.NORMAL)
                self.btn_stop.config(state=tk.NORMAL)
                self.motion_controller.start()
                self.status_var.set("Connected")
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            self.auto_update = False
            self.chk_auto_var.set(False)
            self.robot.disconnect()
            self.motion_controller.stop()
            self.btn_connect.config(text="Connect")
            self.btn_read.config(state=tk.DISABLED)
            self.chk_auto.config(state=tk.DISABLED)
            self.btn_fwd.config(state=tk.DISABLED)
            self.btn_back.config(state=tk.DISABLED)
            self.btn_left.config(state=tk.DISABLED)
            self.btn_right.config(state=tk.DISABLED)
            self.btn_stop.config(state=tk.DISABLED)
            self.status_var.set("Disconnected")
            
    def on_closing(self):
        """Cleanup when the window is closed"""
        self.auto_update = False
        self.motion_controller.stop()
        if self.robot.is_connected:
            self.robot.disconnect()
        self.root.destroy()

    def read_pose_single(self):
        if not self.robot.is_connected:
            return
        self.status_var.set("Reading servos...")
        # Run in a separate thread to avoid freezing the UI
        threading.Thread(target=self._read_pose_task).start()

    def toggle_auto_update(self):
        if self.chk_auto_var.get():
            # Ideally should prevent reading while motion is active,
            # but here we keep it simple and leave it to the user
            # (Auto Update should be OFF during motion).
            self.auto_update = True
            self.status_var.set("Auto updating...")
            threading.Thread(target=self._auto_update_task).start()
        else:
            self.auto_update = False

    def set_motion(self, command):
        self.motion_controller.set_command(command)
        self.status_var.set(f"Motion: {command}")

    def update_motion_params(self, _=None):
        self.motion_controller.stride = self.stride_var.get()
        self.motion_controller.step_height = self.height_var.get()
        self.motion_controller.cycle_time = self.cycle_var.get()

    def trigger_plot_update(self):
        """Safely request a plot update from a thread"""
        try:
            self.root.event_generate("<<UpdatePlot>>", when="tail")
        except Exception:
            pass

    def _auto_update_task(self):
        while self.auto_update and self.robot.is_connected:
            self._read_pose_task()
            time.sleep(0.05) # update interval

    def _read_pose_task(self):
        # Update the robot state
        self.robot.update_pose()
        
        self.trigger_plot_update()

    def _handle_update_plot(self, event):
        self.update_plot()
        self.update_angle_labels()
        self.status_var.set("Updated")

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


class MotionController:
    """Gait motion generation class"""
    def __init__(self, robot_client, on_update_callback):
        self.robot = robot_client
        self.on_update = on_update_callback
        self.running = False
        self.thread = None
        self.command = "stop"
        
        # Gait parameters
        self.cycle_time = 2.0 # Cycle duration [s]
        self.step_height = 10.0 # Step lift height [mm]
        self.stride = 40.0 # Stride [mm]
        self.turn_angle = 15.0 # Turn angle [deg]
        self.body_z = -70.0 # Body height [mm]
        self.base_extension = 210.0 # Base leg radius [mm]

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._loop)
        self.thread.start()

    def stop(self):
        self.running = False
        self.command = "stop"
        if self.thread:
            self.thread.join()

    def set_command(self, cmd):
        self.command = cmd

    def _loop(self):
        start_time = time.time()
        while self.running:
            if not self.robot.is_connected:
                time.sleep(0.1)
                continue

            if self.command == "stop":
                time.sleep(0.1)
                start_time = time.time() # Reset phase
                continue

            elapsed = time.time() - start_time
            phase = (elapsed % self.cycle_time) / self.cycle_time
            
            self._process_gait(phase)
            
            # Send to robot
            self.robot.send_pose_from_kinematics()
            
            # Refresh display
            if self.on_update:
                self.on_update()
            
            time.sleep(0.05) # ~20Hz

    def _process_gait(self, phase):
        sx = 0.0
        rot = 0.0
        
        if self.command == "forward": sx = self.stride
        elif self.command == "backward": sx = -self.stride
        elif self.command == "turn_left": rot = self.turn_angle
        elif self.command == "turn_right": rot = -self.turn_angle

        for i in range(6):
            # Tripod gait (alternating groups of 3 legs)
            # Group A: 0, 2, 4 / Group B: 1, 3, 5
            is_group_a = (i % 2 == 0)
            
            leg_phase = phase
            if not is_group_a:
                leg_phase = (phase + 0.5) % 1.0
            
            x_off = 0.0
            z_off = 0.0
            r_off = 0.0
            
            if leg_phase < 0.5:
                # Swing phase: lift the foot and move it forward
                p = leg_phase * 2.0 # 0.0 -> 1.0
                z_off = np.sin(p * np.pi) * self.step_height
                x_off = -np.cos(p * np.pi) * (sx / 2.0)
                r_off = -np.cos(p * np.pi) * (rot / 2.0)
            else:
                # Stance phase: keep foot on ground and push back
                p = (leg_phase - 0.5) * 2.0 # 0.0 -> 1.0
                z_off = 0.0
                x_off = (sx / 2.0) - (p * sx)
                r_off = (rot / 2.0) - (p * rot)
            
            # Target position in the leg's local coordinate frame
            local_pos = np.array([self.base_extension + x_off, 0.0, self.body_z + z_off])
            
            # Rotation for turning (rotate the local point)
            mat_rot = KumokunKinematics._create_rotation_matrix(0, 0, np.deg2rad(r_off))
            local_pos = KumokunKinematics._transform_point(mat_rot, local_pos)
            
            # Convert to global coordinates (apply leg mount rotation)
            leg = self.robot.kinematics.get_leg(i)
            mat_leg = KumokunKinematics._create_rotation_matrix(0, 0, np.deg2rad(leg.mount_angle_deg))
            abs_pos = KumokunKinematics._transform_point(mat_leg, local_pos)
            
            # Solve IK (this updates the kinematics model state)
            self.robot.kinematics.solve_ik_for_leg(i, abs_pos)


if __name__ == "__main__":
    root = tk.Tk()
    app = VisualizerApp(root)
    root.mainloop()