import time
import sys
import os
from enum import Enum, auto
import threading

# Add current directory to sys.path so local imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import json
from kumokun_servo import KumokunServo
from servo_converter import ServoConverter
from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG, HOME_POSITION, HOMING_DURATION, UPDATE_INTERVAL
from kumokun_state import RobotState
from kumokun_state_homing import HomingManager
from kumokun_state_standup import StandupManager
from kumokun_state_powerup import PowerupManager
from kumokun_state_idle import IdleManager
from kumokun_state_walk_mode0 import WalkMode0Manager
from kumokun_state_walk_mode1 import WalkMode1Manager
from kumokun_state_standdown import StanddownManager
from kumokun_state_powerdown import PowerdownManager
from kumokun_kinematics import KumokunKinematics

class KumokunController:
    def __init__(self, port=None, use_dummy: bool = False):
        # Use the KumokunServo wrapper which selects real or dummy
        self.controller = KumokunServo(use_dummy=use_dummy)
        self.port = port
        self.current_state = RobotState.POWERUP
        self.is_running = True

        # For internal thread: use configured default update interval
        self._update_interval = UPDATE_INTERVAL
        self._thread = None
        self._stop_event = threading.Event()
        # Minimal wait used inside the internal loop to avoid blocking other threads.
        # This is independent of the configured UPDATE_INTERVAL and kept small
        # to reduce latency while preventing a pure busy-spin.
        self._min_sleep = 0.001

        # Manager
        # Shared kinematics instance for use by all state managers
        self.kinematics = KumokunKinematics()

        self.homing_manager = HomingManager(self.controller, kinematics=self.kinematics, update_interval=self._update_interval)
        self.standup_manager = StandupManager(self.controller, kinematics=self.kinematics, update_interval=self._update_interval)
        self.powerup_manager = PowerupManager(self.controller, kinematics=self.kinematics, update_interval=self._update_interval)
        self.idle_manager = IdleManager(self.controller, kinematics=self.kinematics, update_interval=self._update_interval)
        self.walk_mode0_manager = WalkMode0Manager(self.controller, kinematics=self.kinematics, update_interval=self._update_interval)
        self.walk_mode1_manager = WalkMode1Manager(self.controller, kinematics=self.kinematics, update_interval=self._update_interval)
        self.standdown_manager = StanddownManager(self.controller, kinematics=self.kinematics, update_interval=self._update_interval)
        self.powerdown_manager = PowerdownManager(self.controller, kinematics=self.kinematics, update_interval=self._update_interval)

    def connect(self):
        # Attempt to connect using the underlying controller. Some controllers
        # (e.g., the dummy controller) accept a None/empty port and will
        # succeed; allow the controller implementation to decide.
        try:
            print(f"Connecting to {self.port}...")
            success, msg = self.controller.connect(self.port)
            if success:
                print("Connected successfully.")
            else:
                print(f"Failed to connect: {msg}")
            return success, msg
        except Exception as e:
            return False, f"Connection error: {e}"

    def disconnect(self):
        self.stop()
        self.controller.disconnect()
        print("Disconnected.")

    def start(self):
        """Start the internal thread."""
        if self._thread is not None and self._thread.is_alive():
            return

        self.is_running = True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        print("KumokunController thread started.")

    def stop(self):
        """Stop the internal thread."""
        self.is_running = False
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run_loop(self):
        while not self._stop_event.is_set() and self.is_running:
            try:
                self.update()
            except Exception:
                # Ensure the loop continues on unexpected errors
                pass
            # Wait a tiny fixed interval (independent of UPDATE_INTERVAL) so
            # the loop does not busy-spin but also does not block other threads
            # for long periods. Use Event.wait so we wake immediately on stop.
            try:
                self._stop_event.wait(self._min_sleep)
            except Exception:
                pass

    def update(self):
        """State machine update called periodically from the main loop."""
        # Execute per-state handlers
        if self.current_state == RobotState.POWERUP:
            self._handle_powerup()
        elif self.current_state == RobotState.IDLE:
            self._handle_idle()
        elif self.current_state == RobotState.HOMING:
            self._handle_homing()
        elif self.current_state == RobotState.STANDUP:
            self._handle_standup()
        elif self.current_state == RobotState.WALK_MODE0:
            self._handle_walk_mode0()
        elif self.current_state == RobotState.WALK_MODE1:
            self._handle_walk_mode1()
        elif self.current_state == RobotState.STANDDOWN:
            self._handle_standdown()
        elif self.current_state == RobotState.POWERDOWN:
            self._handle_powerdown()

    def change_state(self, new_state):
        """Handle state transition."""
        if self.current_state == new_state:
            return

        print(f"State Transition: {self.current_state.name} -> {new_state.name}")
        
        # Exit actions for previous state
        self._on_exit_state(self.current_state)
        
        # Update state
        self.current_state = new_state
        
        # Entry actions for new state
        self._on_enter_state(self.current_state)

    def process_command(self, command_string):
        """Accept an external command string and attempt a state transition."""
        try:
            new_state = RobotState[command_string.upper()]
            self.change_state(new_state)
            return True, f"Command '{command_string}' processed. Transitioned to {new_state.name}."
        except KeyError:
            return False, f"Invalid command: '{command_string}'. Unknown state."
        except Exception as e:
            return False, f"Error processing command: {str(e)}"

    def is_valid_command(self, command_string):
        """Check whether a command string is a valid state name."""
        try:
            RobotState[command_string.upper()]
            return True
        except KeyError:
            return False
            
    def _on_enter_state(self, state):
        """Actions to perform when entering a state."""
        if state == RobotState.POWERUP:
            print("Entering POWERUP: Starting power-up sequence.")
            self.powerup_manager.start()
        elif state == RobotState.IDLE:
            print("Entering IDLE: Releasing all servos.")
            self.idle_manager.start()
        elif state == RobotState.HOMING:
            print("Entering HOMING: Starting homing sequence.")
            self.homing_manager.start()
        elif state == RobotState.STANDUP:
            print("Entering STANDUP: Starting stand-up sequence.")
            self.standup_manager.start()
        elif state == RobotState.WALK_MODE0:
            print("Entering WALK_MODE0: Starting walk mode 0.")
            self.walk_mode0_manager.start()
        elif state == RobotState.WALK_MODE1:
            print("Entering WALK_MODE1: Starting walk mode 1.")
            self.walk_mode1_manager.start()
        elif state == RobotState.STANDDOWN:
            print("Entering STANDDOWN: Starting sit-down sequence.")
            self.standdown_manager.start()
        elif state == RobotState.POWERDOWN:
            print("Entering POWERDOWN: Starting shutdown sequence.")
            self.powerdown_manager.start()

    def _on_exit_state(self, state):
        """Actions to perform when exiting a state."""
        pass

    # --- State handlers ---

    def _handle_powerup(self):
        # Delegate power-up sequence to manager; transition to IDLE when done
        if not self.powerup_manager.update():
            self.change_state(RobotState.IDLE)

    def _handle_idle(self):
        # Idle handling (background tasks)
        try:
            self.idle_manager.update()
        except Exception:
            # Do not raise from idle updates
            pass

    def _handle_homing(self):
        """HOMING: perform homing sequence"""
        if not self.homing_manager.update():
            print("Homing failed. Returning to IDLE.")
            self.change_state(RobotState.IDLE)

    def _handle_standup(self):
        # Handling while performing stand-up motion
        if not self.standup_manager.update():
            print("Stand-up failed or cancelled. Returning to IDLE.")
            self.change_state(RobotState.IDLE)

    def _handle_walk_mode0(self):
        # Handling for walk mode 0
        if not self.walk_mode0_manager.update():
            self.change_state(RobotState.IDLE)

    def _handle_walk_mode1(self):
        # Handling for walk mode 1
        if not self.walk_mode1_manager.update():
            self.change_state(RobotState.IDLE)

    def _handle_standdown(self):
        # Handling while performing sit-down motion
        # Transition to IDLE when complete
        if not self.standdown_manager.update():
            self.change_state(RobotState.IDLE)

    def _handle_powerdown(self):
        # Shutdown handling
        if not self.powerdown_manager.update():
            print("Powerdown complete; stopping controller.")
            self.stop()
            try:
                self.controller.free_all()
            except Exception:
                pass

    # --- State snapshot APIs ---
    def get_servo_state(self) -> dict:
        """Return a snapshot of the servo state from the KumokunServo wrapper.

        Returns a dict with keys:
        - 'last_positions': dict[int -> Optional[int]] physical_id -> raw_value
        - 'free_states': dict[int -> bool] physical_id -> free
        - 'is_connected': bool
        """
        try:
            last = self.controller.get_last_positions()
        except Exception:
            last = {}
        try:
            free = self.controller.get_free_states()
        except Exception:
            free = {}
        return {
            "last_positions": last,
            "free_states": free,
            "is_connected": bool(getattr(self.controller, "is_connected", False)),
        }

    def get_kinematics_state(self) -> dict:
        """Compute FK from current servo readings using a new KumokunKinematics instance.

        This does NOT use `self.kinematics` for FK results; it creates a fresh
        KumokunKinematics(), sets joint angles inferred from servo raw values,
        runs forward kinematics, and returns per-leg servo angles and world positions.
        """
        # Create fresh kinematics instance
        local_kin = KumokunKinematics()

        # Copy body pose from controller's shared kinematics if available (not using its FK state)
        try:
            local_kin.body_center_pos = getattr(self.kinematics, "body_center_pos").copy()
            local_kin.body_rotation_deg = getattr(self.kinematics, "body_rotation_deg").copy()
        except Exception:
            # leave defaults
            pass

        # Get servo raw positions (physical id -> value)
        servo_state = self.get_servo_state().get("last_positions", {})

        # For each software sid, convert raw value -> degrees and assign to local_kin
        for sid, cfg in SERVO_CONFIG.items():
            phys = cfg.get("physical_sid")
            raw = None
            try:
                raw = servo_state.get(int(phys))
            except Exception:
                raw = None

            angle_deg = None
            if raw is not None:
                try:
                    conv = ServoConverter(direction=cfg.get("direction", 1), offset=cfg.get("offset", 0.0), min_angle=cfg.get("min_angle"), max_angle=cfg.get("max_angle"))
                    angle_deg = float(conv.convert_to_degrees(int(raw)))
                except Exception:
                    angle_deg = None

            # Map software sid to leg/joint
            leg_id = sid // 3
            joint_index = sid % 3
            leg = local_kin.get_leg(leg_id)
            if leg is None:
                continue

            # In SERVO_CONFIG the order per leg is [Link3, Link2, Link1]
            # sid % 3 == 0 -> Link3, 1 -> Link2, 2 -> Link1
            try:
                if joint_index == 0:
                    if angle_deg is not None:
                        leg.link3_servo.ik_angle_deg = angle_deg
                elif joint_index == 1:
                    if angle_deg is not None:
                        leg.link2_servo.ik_angle_deg = angle_deg
                else:
                    if angle_deg is not None:
                        leg.link1_servo.ik_angle_deg = angle_deg
            except Exception:
                pass

        # Run FK to populate absolute positions
        try:
            local_kin.forward_kinematics_all()
        except Exception:
            pass

        # Return the populated KumokunKinematics instance so callers can inspect
        # joint angles and absolute positions directly.
        return local_kin

# Example usage
if __name__ == "__main__":
    import argparse
    import time
    
    parser = argparse.ArgumentParser(description='Kumokun Controller State Machine Test')
    parser.add_argument('port', type=str, help='Serial port (e.g., COM3, /dev/ttyACM0)')
    parser.add_argument('--dummy', action='store_true', help='Use simulated servo controller (no serial)')
    args = parser.parse_args()

    kumokun = KumokunController(port=args.port, use_dummy=args.dummy)
    
    if kumokun.connect()[0]:
        try:
            kumokun.start()
            print("Controller started. Press Ctrl+C to exit.")
            while kumokun.is_running:
                time.sleep(1.0)
                
                # Simple test transition logic (in practice controlled externally)
                # POWERUP -> IDLE is automatic
                # Example: IDLE -> HOMING -> IDLE
                if kumokun.current_state == RobotState.IDLE:
                    cmd = input("Enter command (HOMING, STANDUP, WALK_MODE0, WALK_MODE1, STANDDOWN, POWERDOWN, EXIT): ").strip()
                    
                    if kumokun.is_valid_command(cmd):
                        success, message = kumokun.process_command(cmd)
                        if success and cmd.upper() != "EXIT":
                            print(message)
                        else:
                            print(f"Command failed: {message}")

                    
        except KeyboardInterrupt:
            kumokun.change_state(RobotState.POWERDOWN)
            
        kumokun.disconnect()