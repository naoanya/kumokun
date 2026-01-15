import time
import numpy as np
from typing import Dict
from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG, HOME_POSITION, HOMING_DURATION, UPDATE_INTERVAL


class HomingManager:
    def __init__(self, controller, kinematics: KumokunKinematics = None, update_interval=UPDATE_INTERVAL):
        self.controller = controller
        self.kinematics = kinematics if kinematics is not None else KumokunKinematics()
        self.update_interval = update_interval
        self.start_positions = {}
        self.target_positions = {}
        self.start_time = 0.0
        self.duration = HOMING_DURATION
        self.state = "IDLE"
        self.last_update_time = 0.0
        # constants
        self.NUM_LEGS = 6
        self.SERVOS_PER_LEG = 3
        self.TOTAL_SERVOS = self.NUM_LEGS * self.SERVOS_PER_LEG

    def _read_start_positions(self) -> bool:
        """Release servos and populate `self.start_positions` from controller snapshot."""
        self.start_positions = {}
        ok, msg = self.controller.free_all()
        if not ok:
            print(f"Error getting positions: {msg}")
            return False

        positions = self.controller.get_last_positions()
        for sid, pos in positions.items():
            if pos is not None:
                self.start_positions[sid] = pos
        return True

    def _calculate_target_positions(self) -> None:
        """Compute target servo values for the configured home position."""
        kinematics = self.kinematics
        self.target_positions = {}

        HOME_X = HOME_POSITION["x"]
        HOME_Y = HOME_POSITION["y"]
        HOME_Z = HOME_POSITION["z"]

        converters = getattr(self.controller, 'converters', None)
        if converters is None:
            print("Error: controller.converters missing")
            return
        expected_sids = set(range(self.TOTAL_SERVOS))
        missing = sorted(expected_sids - set(converters.keys()))
        if missing:
            print(f"Missing SERVO_CONFIG for sid(s): {missing}")
            return

        for leg_id in range(self.NUM_LEGS):
            leg = kinematics.get_leg(leg_id)

            mat = KumokunKinematics._create_rotation_matrix(0, 0, np.deg2rad(leg.mount_angle_deg))
            local_pos = np.array([HOME_X, HOME_Y, HOME_Z])
            abs_pos = KumokunKinematics._transform_point(mat, local_pos)

            ret = kinematics.solve_ik_for_leg(leg_id, abs_pos)
            if ret != 0:
                print(f"Warning: IK failed for Leg {leg_id}")
                continue

            base_sid = leg_id * self.SERVOS_PER_LEG
            # Map computed angles (link3, link2, link1) to servo target values
            self._assign_target(base_sid + 0, leg.link3_servo.ik_angle_deg, converters)
            self._assign_target(base_sid + 1, leg.link2_servo.ik_angle_deg, converters)
            self._assign_target(base_sid + 2, leg.link1_servo.ik_angle_deg, converters)

    def _assign_target(self, software_sid: int, angle_deg: float, converters: Dict[int, object]) -> None:
        """Convert an IK angle to servo raw value and store in target_positions.

        software_sid: logical sid (0..TOTAL_SERVOS-1)
        angle_deg: angle in degrees computed by IK
        converters: mapping software_sid -> ServoConverter-like object
        """
        try:
            conf = SERVO_CONFIG[software_sid]
            phys = conf["physical_sid"]
            conv = converters[software_sid]
            self.target_positions[phys] = conv.convert_to_value(angle_deg)
        except Exception as e:
            print(f"Servo conversion error for sid {software_sid}: {e}")

    def start(self):
        self.state = "ENTER"

    def update(self):
        if self.state == "ENTER":
            if self.prepare():
                self.start_move()
                self.state = "MOVE"
            else:
                self.state = "IDLE"
                return False
        elif self.state == "MOVE":
            if self.update_move():
                self.state = "DONE"
        elif self.state == "DONE":
            pass
        return True

    def prepare(self):
        """HOMING_ENTER: Read current positions and calculate target positions"""
        print("Releasing servos and reading current positions...")
        if not self._read_start_positions():
            return False

        print(f"Read positions for {len(self.start_positions)} servos.")
        print(f"Calculating target positions (Home: X={HOME_POSITION['x']}, Y={HOME_POSITION['y']}, Z={HOME_POSITION['z']})...")
        
        try:
            self._calculate_target_positions()
        except Exception as e:
            print(f"Error calculating target positions: {e}")

        return True

    def start_move(self):
        self.start_time = time.time()
        self.last_update_time = self.start_time - self.update_interval

    def update_move(self):
        try:
            current_time = time.time()
            if current_time - self.last_update_time < self.update_interval:
                return False
            self.last_update_time = current_time

            elapsed = current_time - self.start_time
            progress = elapsed / self.duration
            if progress > 1.0:
                progress = 1.0
            
            send_values = []
            for sid in range(1, self.TOTAL_SERVOS + 1):
                start_val = self.start_positions.get(sid, 7500)
                target_val = self.target_positions.get(sid, start_val)
                curr_val = int(start_val + (target_val - start_val) * progress)
                send_values.append(curr_val)
            
            self.controller.set_all_pos(send_values)
            
            return progress >= 1.0
        except Exception as e:
            print(f"Error calculating target positions: {e}")
            return False
