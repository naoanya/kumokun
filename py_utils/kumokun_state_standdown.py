import time
import json
import numpy as np

from servo_converter import ServoConverter
from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG, HOME_POSITION, HOMING_DURATION, UPDATE_INTERVAL


class StanddownManager:
    """Manager for STANDDOWN: smoothly move body to a lower height.

    This mirrors StandupManager but targets a lower body Z (sit/down).
    """

    def __init__(self, controller, kinematics: KumokunKinematics = None, target_body_z: float = -120.0, update_interval: float = UPDATE_INTERVAL, duration: float = HOMING_DURATION):
        self.controller = controller
        self.kinematics = kinematics if kinematics is not None else KumokunKinematics()
        self.update_interval = update_interval
        # converters obtained from `self.controller.converters` when needed
        self.start_positions = {}
        self.target_positions = {}
        self.start_time = 0.0
        self.duration = duration
        self.state = "IDLE"
        self.last_update_time = 0.0
        self.target_body_z = target_body_z

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
        responses, err = self.controller.free_all()
        if err:
            print(f"Error getting positions: {err}")
            return False

        self.start_positions = {}
        for resp in responses:
            try:
                data = json.loads(resp)
                sid = data.get("id")
                pos = data.get("pos", data.get("feedback"))
                if sid is not None and pos is not None:
                    self.start_positions[sid] = pos
            except json.JSONDecodeError:
                pass

        kinematics = self.kinematics
        self.target_positions = {}

        converters = getattr(self.controller, 'converters', None)
        if not converters:
            converters = {}
            for sid, conf in SERVO_CONFIG.items():
                converters[sid] = ServoConverter(conf["direction"], conf["offset"], min_angle=conf.get("min_angle"), max_angle=conf.get("max_angle"))

        HOME_X = HOME_POSITION.get("x", 200.0)
        HOME_Y = HOME_POSITION.get("y", 0.0)

        for leg_id in range(6):
            leg = kinematics.get_leg(leg_id)
            mat = KumokunKinematics._create_rotation_matrix(0, 0, np.deg2rad(leg.mount_angle_deg))
            local_pos = np.array([HOME_X, HOME_Y, self.target_body_z])
            world_pos = KumokunKinematics._transform_point(mat, local_pos)

            ret = kinematics.solve_ik_for_leg(leg_id, world_pos)
            if ret != 0:
                print(f"Warning: IK failed for Leg {leg_id} during stand-down target calc")
                continue

            base_sid = leg_id * 3
            sid = base_sid + 0
            conf = SERVO_CONFIG[sid]
            try:
                self.target_positions[conf["physical_sid"]] = converters[sid].convert_to_value(leg.link3_servo.ik_angle_deg)
            except Exception as e:
                print(f"Servo conversion error for sid {sid}: {e}")

            sid = base_sid + 1
            conf = SERVO_CONFIG[sid]
            try:
                self.target_positions[conf["physical_sid"]] = converters[sid].convert_to_value(leg.link2_servo.ik_angle_deg)
            except Exception as e:
                print(f"Servo conversion error for sid {sid}: {e}")

            sid = base_sid + 2
            conf = SERVO_CONFIG[sid]
            try:
                self.target_positions[conf["physical_sid"]] = converters[sid].convert_to_value(leg.link1_servo.ik_angle_deg)
            except Exception as e:
                print(f"Servo conversion error for sid {sid}: {e}")

        return True

    def start_move(self):
        self.start_time = time.time()
        self.last_update_time = self.start_time - self.update_interval

    def update_move(self):
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return False
        self.last_update_time = current_time

        elapsed = current_time - self.start_time
        progress = elapsed / self.duration
        if progress > 1.0:
            progress = 1.0

        send_values = []
        for sid in range(1, 19):
            start_val = self.start_positions.get(sid, 7500)
            target_val = self.target_positions.get(sid, start_val)
            curr_val = int(start_val + (target_val - start_val) * progress)
            send_values.append(curr_val)

        self.controller.set_all_pos(send_values)

        return progress >= 1.0
