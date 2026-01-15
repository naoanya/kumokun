#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kumo-kun Kinematics Visualizer
Application to verify inverse kinematics (IK) and walking motions without hardware
"""
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import os
import threading
import time
import math

# Add py_utils to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
py_utils_path = os.path.join(os.path.dirname(current_dir), 'py_utils')
if py_utils_path not in sys.path:
    sys.path.append(py_utils_path)

from kumokun_kinematics import KumokunKinematics
from kumokun_config import KINEMATICS_CONFIG, HOME_POSITION, SERVO_CONFIG
from servo_converter import ServoConverter, ServoConversionError

class VirtualRobot:
    """Manage the kinematics model without hardware communication."""
    def __init__(self):
        self.kinematics = KumokunKinematics()
        self.ik_status = [0] * 6
        self.servo_converters = self._create_servo_converters()
        self.reset_pose()

    def reset_pose(self):
        """Reset pose to the default state."""
        # Re-initialize kinematics (compatible with v2 which removed setup())
        self.kinematics = KumokunKinematics()
        self.ik_status = [0] * 6
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

    def _compute_target_world(self, leg_id: int, foot_offset: np.ndarray) -> np.ndarray:
        """Compute target toe position in world coordinates for a leg."""
        leg = self.kinematics.get_leg(leg_id)
        home_pos_leg_local = np.array([
            HOME_POSITION["x"],
            HOME_POSITION["y"],
            HOME_POSITION["z"]
        ])
        target_pos_leg_local = home_pos_leg_local + foot_offset

        leg_mount_rad = math.radians(leg.mount_angle_deg)
        leg_mount_rot_matrix = KumokunKinematics._create_rotation_matrix(0, 0, leg_mount_rad)
        return KumokunKinematics._transform_point(leg_mount_rot_matrix, target_pos_leg_local)

    def _validate_servo_conversion(self, leg_id: int) -> bool:
        """Validate IK angles against ServoConverter limits."""
        base_sid = leg_id * 3
        sid_link3 = base_sid + 0
        sid_link2 = base_sid + 1
        sid_link1 = base_sid + 2

        leg = self.kinematics.get_leg(leg_id)

        try:
            self.servo_converters[sid_link3].convert_to_value(leg.link3_servo.ik_angle_deg)
            self.servo_converters[sid_link2].convert_to_value(leg.link2_servo.ik_angle_deg)
            self.servo_converters[sid_link1].convert_to_value(leg.link1_servo.ik_angle_deg)
        except ServoConversionError:
            return False

        return True

    def update_ik(self, body_pos, body_rot, foot_offsets):
        """
        Update IK calculation for all legs.
        
        Coordinate System:
        - World Coordinates: Fixed global reference frame. Unaffected by robot configuration.
        - Body Configuration: body_center_pos and body_rotation_deg are set via kinematics object.
        
        :param body_pos: [x, y, z] body center position in world coordinates (mm)
        :param body_rot: [roll, pitch, yaw] body rotation in degrees
        :param foot_offsets: list of 6 [x, y, z] foot offsets (in leg-local frame)
        """
        # Set body configuration
        self.kinematics.body_center_pos = np.array(body_pos)
        self.kinematics.body_rotation_deg = np.array(body_rot)

        # Calculate target positions for each leg in world coordinates
        for i in range(6):
            target_pos_world = self._compute_target_world(i, foot_offsets[i])

            # Call IK solver with world coordinates
            # The kinematics solver will internally transform world coordinates to body frame
            # using body_center_pos, body_rotation_deg, and default_height
            ret = self.kinematics.solve_ik_for_leg(i, target_pos_world)

            conversion_ok = (ret == 0) and self._validate_servo_conversion(i)
            self.ik_status[i] = 0 if conversion_ok else -1

class KinematicsVisualizerApp:
    # UI Configuration Constants
    WINDOW_WIDTH = 1200
    WINDOW_HEIGHT = 800
    PLOT_LIMIT = 250
    PLOT_Z_MIN = -100
    PLOT_Z_MAX = 200
    AXIS_LENGTH = 40
    LEG_COUNT = 6
    
    # View angles
    VIEW_TOP = (90, -90)
    VIEW_FRONT = (20, -90)
    VIEW_SIDE = (20, 180)
    
    # Body control ranges
    BODY_POS_RANGE = (-50, 50)
    BODY_ROT_RANGE = (-20, 20)
    LEG_OFFSET_RANGE = (-150, 150)
    MOTION_SPEED_RANGE = (0.2, 3.0)
    
    def __init__(self, root):
        self.root = root
        self.root.title("Kumo-kun Kinematics Visualizer (No Hardware)")
        self.root.geometry(f"{self.WINDOW_WIDTH}x{self.WINDOW_HEIGHT}")
        
        self.robot = VirtualRobot()
        self.motion_controller = VirtualMotionController(self.robot, self.trigger_plot_update)
        
        # Per-leg control state
        self.leg_offsets = np.zeros((6, 3)) # [LegID][x, y, z]
        self.selected_leg = tk.IntVar(value=0)

        self.setup_ui()
        
        # Initial drawing
        self.update_plot()

        # Event bindings
        self.root.bind("<<UpdatePlot>>", self._handle_update_plot)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        # Left: Control panel (container)
        self.control_panel = ttk.Frame(self.root, padding=10, width=320)
        self.control_panel.pack(side=tk.LEFT, fill=tk.Y)
        
        # Create tab control
        self.notebook = ttk.Notebook(self.control_panel)
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Tab 1: Body & Motion
        self.tab_main = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_main, text="Body & Motion")
        self._setup_body_control(self.tab_main)
        self._setup_motion_group(self.tab_main)
        
        # Tab 2: Single Leg IK
        self.tab_leg = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_leg, text="Single Leg")
        self._setup_single_leg_control(self.tab_leg)

        # Common display: angle readouts (placed below the Notebook)
        self._setup_angle_display(self.control_panel)

        # View Control
        self._setup_view_control(self.control_panel)

        # Right: 3D display
        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self._setup_plot_area(self.plot_frame)

    def _setup_body_control(self, parent):
        group = ttk.LabelFrame(parent, text="Body IK Control", padding=10)
        group.pack(fill=tk.X, pady=5)
        
        # Body Translation
        ttk.Label(group, text="Body Position (X, Y, Z)").pack(anchor=tk.W)
        self.var_body_x = self._create_slider(group, self.BODY_POS_RANGE[0], self.BODY_POS_RANGE[1], 0, self.on_ik_change)
        self.var_body_y = self._create_slider(group, self.BODY_POS_RANGE[0], self.BODY_POS_RANGE[1], 0, self.on_ik_change)
        self.var_body_z = self._create_slider(group, self.BODY_POS_RANGE[0], self.BODY_POS_RANGE[1], 0, self.on_ik_change)
        
        # Body Rotation
        ttk.Label(group, text="Body Rotation (Roll, Pitch, Yaw)").pack(anchor=tk.W)
        self.var_body_r = self._create_slider(group, self.BODY_ROT_RANGE[0], self.BODY_ROT_RANGE[1], 0, self.on_ik_change)
        self.var_body_p = self._create_slider(group, self.BODY_ROT_RANGE[0], self.BODY_ROT_RANGE[1], 0, self.on_ik_change)
        self.var_body_yw = self._create_slider(group, self.BODY_ROT_RANGE[0], self.BODY_ROT_RANGE[1], 0, self.on_ik_change)

        ttk.Button(group, text="Reset Pose", command=self.reset_sliders).pack(fill=tk.X, pady=5)

    def _create_slider(self, parent, min_val, max_val, default, callback):
        var = tk.DoubleVar(value=default)
        s = ttk.Scale(parent, from_=min_val, to=max_val, variable=var, command=lambda v: callback())
        s.pack(fill=tk.X)
        return var

    def _setup_motion_group(self, parent):
        group = ttk.LabelFrame(parent, text="Motion Simulation", padding=10)
        group.pack(fill=tk.X, pady=5)
        
        btn_frame = ttk.Frame(group)
        btn_frame.pack(fill=tk.X)
        
        # Define motion buttons layout: (text, command, row, col)
        motion_buttons = [
            ("Fwd", "forward", 0, 1),
            ("Left", "turn_left", 1, 0),
            ("Stop", "stop", 1, 1),
            ("Right", "turn_right", 1, 2),
            ("Back", "backward", 2, 1)
        ]
        
        for text, cmd, row, col in motion_buttons:
            ttk.Button(btn_frame, text=text, command=lambda c=cmd: self.set_motion(c)).grid(row=row, column=col)

        # Speed parameter
        param_frame = ttk.Frame(group)
        param_frame.pack(fill=tk.X, pady=5)
        ttk.Label(param_frame, text="Speed:").pack(anchor=tk.W)
        self.cycle_var = tk.DoubleVar(value=self.motion_controller.DEFAULT_CYCLE_TIME)
        ttk.Scale(param_frame, from_=self.MOTION_SPEED_RANGE[0], 
                 to=self.MOTION_SPEED_RANGE[1], variable=self.cycle_var, 
                 command=self.update_motion_params).pack(fill=tk.X)

    def _setup_single_leg_control(self, parent):
        # Leg selection frame
        sel_frame = ttk.LabelFrame(parent, text="Select Leg", padding=5)
        sel_frame.pack(fill=tk.X, pady=5)
        
        # Create 2x3 grid of radio buttons
        for i in range(self.LEG_COUNT):
            btn = ttk.Radiobutton(sel_frame, text=f"Leg {i}", variable=self.selected_leg, 
                                 value=i, command=self._on_leg_selection_change)
            btn.grid(row=i//3, column=i%3, sticky=tk.W, padx=5, pady=2)
            
        # Foot offset sliders
        group = ttk.LabelFrame(parent, text="Foot Offset (Local)", padding=10)
        group.pack(fill=tk.X, pady=5)
        
        # Create offset sliders for each axis
        offset_axes = [("X Offset", "var_leg_x"), ("Y Offset", "var_leg_y"), ("Z Offset", "var_leg_z")]
        for label, var_name in offset_axes:
            ttk.Label(group, text=label).pack(anchor=tk.W)
            slider_var = self._create_slider(group, self.LEG_OFFSET_RANGE[0], 
                                            self.LEG_OFFSET_RANGE[1], 0, self.on_single_leg_change)
            setattr(self, var_name, slider_var)
        
        # Reset buttons
        btn_frame = ttk.Frame(group)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="Reset Leg", command=self.reset_current_leg).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(btn_frame, text="Reset All", command=self.reset_all_legs).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=2)

    def _on_leg_selection_change(self):
        # Reflect the selected leg's current offset on the sliders
        leg_id = self.selected_leg.get()
        offsets = self.leg_offsets[leg_id]
        self.var_leg_x.set(offsets[0])
        self.var_leg_y.set(offsets[1])
        self.var_leg_z.set(offsets[2])

    def on_single_leg_change(self):
        # Save slider values to the current leg's offset
        leg_id = self.selected_leg.get()
        self.leg_offsets[leg_id] = [self.var_leg_x.get(), self.var_leg_y.get(), self.var_leg_z.get()]
        self.on_ik_change()

    def reset_current_leg(self):
        leg_id = self.selected_leg.get()
        self.leg_offsets[leg_id] = [0, 0, 0]
        self._on_leg_selection_change() # Reset sliders
        self.on_ik_change()

    def reset_all_legs(self):
        self.leg_offsets = np.zeros((6, 3))
        self._on_leg_selection_change()
        self.on_ik_change()

    def _setup_angle_display(self, parent):
        angle_group = ttk.LabelFrame(parent, text="Servo Angles (Deg)", padding=5)
        angle_group.pack(fill=tk.X, pady=5)
        
        headers = ["Leg", "Link3", "Link2", "Link1"]
        for col, text in enumerate(headers):
            ttk.Label(angle_group, text=text, font=("", 8, "bold")).grid(row=0, column=col, padx=2, pady=2)
            
        self.angle_labels = {}
        for i in range(6):
            ttk.Label(angle_group, text=f"L{i}", font=("", 8)).grid(row=i+1, column=0, padx=2, pady=1)
            for j, part in enumerate(["Link3", "Link2", "Link1"]):
                lbl = ttk.Label(angle_group, text="0.0", width=6, anchor="e", font=("", 8))
                lbl.grid(row=i+1, column=j+1, padx=2, pady=1)
                self.angle_labels[f"L{i}_{part}"] = lbl

    def _setup_view_control(self, parent):
        group = ttk.LabelFrame(parent, text="View Control", padding=10)
        group.pack(fill=tk.X, pady=5)
        
        btn_frame = ttk.Frame(group)
        btn_frame.pack(fill=tk.X)
        
        # Define view buttons using constants
        views = [
            ("Top", self.VIEW_TOP),
            ("Front", self.VIEW_FRONT),
            ("Side", self.VIEW_SIDE)
        ]
        
        for label, (elev, azim) in views:
            ttk.Button(btn_frame, text=label, 
                      command=lambda e=elev, a=azim: self.set_view(e, a)).pack(
                      side=tk.LEFT, expand=True, fill=tk.X, padx=2)

    def _setup_plot_area(self, parent):
        self.fig = plt.Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def reset_sliders(self):
        self.var_body_x.set(0)
        self.var_body_y.set(0)
        self.var_body_z.set(0)
        self.var_body_r.set(0)
        self.var_body_p.set(0)
        self.var_body_yw.set(0)
        self.on_ik_change()

    def on_ik_change(self):
        # Ignore manual controls while motion playback is active
        is_motion_running = self.motion_controller.running and self.motion_controller.command != "stop"
        if is_motion_running:
            return

        body_pos = [self.var_body_x.get(), self.var_body_y.get(), self.var_body_z.get()]
        body_rot = [self.var_body_r.get(), self.var_body_p.get(), self.var_body_yw.get()]
        
        # Use per-leg offsets
        foot_offsets = self.leg_offsets

        self.robot.update_ik(body_pos, body_rot, foot_offsets)
        self.update_plot()
        self.update_angle_labels()

    def set_motion(self, command):
        if command == "stop":
            self.motion_controller.set_command("stop")
        else:
            self.motion_controller.start()
            self.motion_controller.set_command(command)

    def update_motion_params(self, _=None):
        self.motion_controller.cycle_time = self.cycle_var.get()

    def set_view(self, elev, azim):
        """Sets the camera view for the 3D plot."""
        self.ax.view_init(elev=elev, azim=azim)
        self.canvas.draw()

    def trigger_plot_update(self):
        try:
            self.root.event_generate("<<UpdatePlot>>", when="tail")
        except Exception:
            pass

    def _handle_update_plot(self, event):
        self.update_plot()
        self.update_angle_labels()

    def update_angle_labels(self):
        """Update servo angle labels for all legs."""
        link_names = ["Link1", "Link2", "Link3"]
        servo_attrs = ["link1_servo", "link2_servo", "link3_servo"]
        
        for i in range(self.LEG_COUNT):
            leg = self.robot.kinematics.get_leg(i)
            if leg:
                for link_name, servo_attr in zip(link_names, servo_attrs):
                    servo = getattr(leg, servo_attr)
                    self.angle_labels[f"L{i}_{link_name}"].config(
                        text=f"{servo.ik_angle_deg:.1f}")

    def update_plot(self):
        """Updates the 3D plot with current robot state."""
        current_azim = getattr(self.ax, 'azim', None)
        current_elev = getattr(self.ax, 'elev', None)
        self.ax.clear()
        
        # Setup plot bounds and labels
        self._setup_plot_bounds()
        
        # Get body origin
        default_height = KINEMATICS_CONFIG["lBodyHeight"]
        body_origin = self.robot.kinematics.body_center_pos + np.array([0.0, 0.0, default_height])
        
        # Draw floor
        self._draw_floor()
        
        # Draw legs
        self._draw_legs(body_origin)
        
        # Draw body axes
        self._draw_body_axes(body_origin)
        
        # Draw motion direction indicator
        self._draw_motion_indicator(body_origin)
        
        # Restore view angle
        if current_azim is not None:
            self.ax.view_init(elev=current_elev, azim=current_azim)
        self.canvas.draw()
    
    def _setup_plot_bounds(self):
        """Setup 3D plot bounds and labels."""
        self.ax.set_xlim(-self.PLOT_LIMIT, self.PLOT_LIMIT)
        self.ax.set_ylim(-self.PLOT_LIMIT, self.PLOT_LIMIT)
        self.ax.set_zlim(self.PLOT_Z_MIN, self.PLOT_Z_MAX)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
    
    def _draw_floor(self):
        """Draw the floor surface."""
        xx, yy = np.meshgrid(np.linspace(-200, 200, 2), np.linspace(-200, 200, 2))
        self.ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.1, color='gray')
    
    def _draw_legs(self, body_origin):
        """Draw all legs with FK results."""
        colors = ['b', 'g', 'c', 'm', 'y', 'k']  # Red is reserved for error
        
        for i in range(self.LEG_COUNT):
            leg = self.robot.kinematics.get_leg(i)
            
            # Get joint positions
            p_origin = body_origin
            p_link1 = leg.link1_servo.absolute_position
            p_link2 = leg.link2_servo.absolute_position
            p_link3 = leg.link3_servo.absolute_position
            p_toe = leg.end_effector.absolute_position
            
            xs = [p_origin[0], p_link1[0], p_link2[0], p_link3[0], p_toe[0]]
            ys = [p_origin[1], p_link1[1], p_link2[1], p_link3[1], p_toe[1]]
            zs = [p_origin[2], p_link1[2], p_link2[2], p_link3[2], p_toe[2]]
            
            # Color and linewidth based on IK success
            color = colors[i]
            lw = 1.0
            if self.robot.ik_status[i] != 0:
                color = 'r'
                lw = 2.0
            
            self.ax.plot(xs, ys, zs, marker='o', color=color, markersize=4, linewidth=lw)
    
    def _draw_body_axes(self, body_origin):
        """Draw body-frame coordinate axes."""
        body_rot_deg = self.robot.kinematics.body_rotation_deg
        rot_mat = KumokunKinematics._create_rotation_matrix(
            math.radians(body_rot_deg[0]), math.radians(body_rot_deg[1]), math.radians(body_rot_deg[2])
        )
        
        # X-axis (red), Y-axis (green), Z-axis (blue)
        axes_vectors = [
            (np.array([self.AXIS_LENGTH, 0, 0]), 'r'),
            (np.array([0, self.AXIS_LENGTH, 0]), 'g'),
            (np.array([0, 0, self.AXIS_LENGTH]), 'b')
        ]
        
        for axis_vec, color in axes_vectors:
            rotated = rot_mat @ axis_vec
            self.ax.quiver(body_origin[0], body_origin[1], body_origin[2],
                          rotated[0], rotated[1], rotated[2],
                          color=color, arrow_length_ratio=0.15)
    
    def _draw_motion_indicator(self, body_origin):
        """Draw arrow indicating current motion direction."""
        cmd = self.motion_controller.command
        if cmd == "stop":
            return
        
        # Arrow start position (body center + offset upward)
        ox, oy, oz = body_origin[0], body_origin[1], body_origin[2] + 60
        u, v, w = 0, 0, 0
        
        # Determine arrow direction based on motion command
        if cmd == "forward":
            u = 80
        elif cmd == "backward":
            u = -80
        elif cmd == "turn_left":
            ox += 30
            v = 60
        elif cmd == "turn_right":
            ox += 30
            v = -60
        
        if u != 0 or v != 0:
            self.ax.quiver(ox, oy, oz, u, v, w, color='orange', linewidth=3, arrow_length_ratio=0.2)

    def on_closing(self):
        self.motion_controller.stop()
        self.root.destroy()

class VirtualMotionController:
    """Perform gait IK calculations without hardware"""
    # Gait parameters (can be tuned)
    DEFAULT_CYCLE_TIME = 2.0
    DEFAULT_STEP_HEIGHT = 20.0
    DEFAULT_STRIDE = 40.0
    DEFAULT_TURN_ANGLE = 15.0
    DEFAULT_BASE_EXTENSION = 210.0
    
    # Gait phases
    PHASE_SWING = 0.5
    UPDATE_INTERVAL = 0.05
    
    def __init__(self, robot, on_update_callback):
        self.robot = robot
        self.on_update = on_update_callback
        self.running = False
        self.thread = None
        self.command = "stop"
        
        self.cycle_time = self.DEFAULT_CYCLE_TIME
        self.step_height = self.DEFAULT_STEP_HEIGHT
        self.stride = self.DEFAULT_STRIDE
        self.turn_angle = self.DEFAULT_TURN_ANGLE
        self.base_extension = self.DEFAULT_BASE_EXTENSION

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        self.command = "stop"
        if self.thread:
            self.thread.join(timeout=1.0)
            self.thread = None

    def set_command(self, cmd):
        self.command = cmd

    def _loop(self):
        start_time = time.time()
        while self.running:
            if self.command == "stop":
                time.sleep(0.1)
                start_time = time.time()
                continue

            elapsed = time.time() - start_time
            phase = (elapsed % self.cycle_time) / self.cycle_time
            
            self._process_gait(phase)
            
            if self.on_update:
                self.on_update()
            
            time.sleep(0.05)

    def _process_gait(self, phase):
        """Process gait calculation for current phase."""
        sx = 0.0
        rot = 0.0
        
        # Determine stride and rotation based on command
        if self.command == "forward":
            sx = self.stride
        elif self.command == "backward":
            sx = -self.stride
        elif self.command == "turn_left":
            rot = self.turn_angle
        elif self.command == "turn_right":
            rot = -self.turn_angle

        # Calculate foot offsets for all legs
        foot_offsets = []
        for i in range(6):
            offset = self._calculate_leg_offset(i, phase, sx, rot)
            foot_offsets.append(offset)

        # Update IK with new foot positions
        self.robot.update_ik([0, 0, 0], [0, 0, 0], foot_offsets)
    
    def _calculate_leg_offset(self, leg_id, phase, stride, turn_angle):
        """
        Calculate foot offset for a single leg during gait cycle.
        
        :param leg_id: Leg index (0-5)
        :param phase: Gait phase (0-1)
        :param stride: Forward stride distance
        :param turn_angle: Turn angle in degrees
        :return: [x, y, z] offset for the leg
        """
        # Alternate legs: group A (0,2,4) and group B (1,3,5)
        is_group_a = (leg_id % 2 == 0)
        leg_phase = phase if is_group_a else (phase + 0.5) % 1.0
        
        # Calculate swing and stance motions
        z_off, x_off, r_off = self._calculate_leg_motion(leg_phase, stride, turn_angle)
        
        # Transform to absolute coordinates
        leg = self.robot.kinematics.get_leg(leg_id)
        leg_rad = math.radians(leg.mount_angle_deg)
        
        # Rotation-induced lateral motion
        dy_rot = self.base_extension * np.sin(np.deg2rad(r_off))
        
        # Combine forward and rotational components
        abs_off_x = x_off - math.sin(leg_rad) * dy_rot
        abs_off_y = 0.0 + math.cos(leg_rad) * dy_rot
        
        return [abs_off_x, abs_off_y, z_off]
    
    def _calculate_leg_motion(self, leg_phase, stride, turn_angle):
        """
        Calculate individual leg motion (swing/stance).
        
        :param leg_phase: Phase for this leg (0-1)
        :param stride: Forward stride distance
        :param turn_angle: Turn angle in degrees
        :return: (z_offset, x_offset, rotation_offset)
        """
        if leg_phase < self.PHASE_SWING:
            # Swing phase: leg lifted and moved forward
            p = leg_phase * 2.0
            z_off = np.sin(p * np.pi) * self.step_height
            x_off = -np.cos(p * np.pi) * (stride / 2.0)
            r_off = -np.cos(p * np.pi) * (turn_angle / 2.0)
        else:
            # Stance phase: leg on ground, body moves forward
            p = (leg_phase - self.PHASE_SWING) * 2.0
            z_off = 0.0
            x_off = (stride / 2.0) - (p * stride)
            r_off = (turn_angle / 2.0) - (p * turn_angle)
        
        return z_off, x_off, r_off

if __name__ == "__main__":
    root = tk.Tk()
    app = KinematicsVisualizerApp(root)
    root.mainloop()