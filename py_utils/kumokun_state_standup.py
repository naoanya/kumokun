import time
import numpy as np
from typing import Dict

from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG, STANDUP_POSITION, STANDUP_SPEED, HOMING_DURATION, UPDATE_INTERVAL


class StandupManager:
    """Standup manager that moves each toe along a trajectory in two phases:

    - MOVE1: move toe from current position to ground contact target (STANDUP_POSITION z forced to 0)
    - MOVE2: move toe from contact position to final stand position (STANDUP_POSITION)

    Movement is performed in world coordinates at a linear speed `STANDUP_SPEED`.
    """

    def __init__(self, controller, kinematics: KumokunKinematics = None, update_interval: float = UPDATE_INTERVAL, duration: float = HOMING_DURATION):
        self.controller = controller
        self.kinematics = kinematics if kinematics is not None else KumokunKinematics()
        self.update_interval = update_interval

        # Per-leg state
        self.start_toe_pos: Dict[int, np.ndarray] = {}
        self.target1_toe_pos: Dict[int, np.ndarray] = {}
        self.target2_toe_pos: Dict[int, np.ndarray] = {}

        # Track which phase we're in: 'IDLE' | 'ENTER' | 'MOVE1' | 'MOVE2' | 'DONE'
        self.state = 'IDLE'
        self.start_time = 0.0
        self.last_update_time = 0.0
        self.duration = duration

        # constants
        self.NUM_LEGS = 6
        self.SERVOS_PER_LEG = 3
        self.TOTAL_SERVOS = self.NUM_LEGS * self.SERVOS_PER_LEG

    def start(self):
        self.state = 'ENTER'

    def update(self):
        if self.state == 'ENTER':
            if not self.prepare():
                self.state = 'IDLE'
                return False
            self.start_move()
            self.state = 'MOVE1'
        elif self.state == 'MOVE1':
            if self.update_move(stage=1):
                # proceed to MOVE2
                self.state = 'MOVE2'
                # reset timing
                self.start_time = time.time()
                self.last_update_time = self.start_time - self.update_interval
        elif self.state == 'MOVE2':
            if self.update_move(stage=2):
                self.state = 'DONE'
        elif self.state == 'DONE':
            pass
        return True

    def prepare(self) -> bool:
        """Read current toe positions and compute targets for MOVE1 and MOVE2.

        Requires `controller.converters` to be present for sending servo values.
        """
        print('Preparing stand-up: reading current kinematics...')

        # Refresh FK on the shared kinematics to get up-to-date toe positions
        try:
            self.kinematics.forward_kinematics_all()
        except Exception:
            pass

        converters = getattr(self.controller, 'converters', None)
        if converters is None:
            print('Error: controller.converters missing')
            return False

        # Build per-leg start and target positions
        local_target_full = np.array([STANDUP_POSITION['x'], STANDUP_POSITION['y'], STANDUP_POSITION['z']])
        local_target_contact = np.array([STANDUP_POSITION['x'], STANDUP_POSITION['y'], 0.0])

        for leg_id in range(self.NUM_LEGS):
            leg = self.kinematics.get_leg(leg_id)
            if leg is None:
                continue

            # Current toe absolute position
            cur_toe = leg.end_effector.absolute_position.copy()
            self.start_toe_pos[leg_id] = cur_toe

            # Compute world-space target positions by rotating local target by mount angle
            mat_contact = KumokunKinematics._create_rotation_matrix(0, 0, np.deg2rad(leg.mount_angle_deg))
            t1 = KumokunKinematics._transform_point(mat_contact, local_target_contact)
            t2 = KumokunKinematics._transform_point(mat_contact, local_target_full)
            self.target1_toe_pos[leg_id] = t1
            self.target2_toe_pos[leg_id] = t2

        print(f'Stand-up targets prepared for {len(self.start_toe_pos)} legs.')
        return True

    def start_move(self):
        self.start_time = time.time()
        self.last_update_time = self.start_time - self.update_interval

    def update_move(self, stage: int) -> bool:
        """Perform movement for the given stage (1 or 2).

        Returns True when the stage is complete for all legs.
        """
        now = time.time()
        if now - self.last_update_time < self.update_interval:
            return False
        dt = now - self.last_update_time
        self.last_update_time = now

        speed = STANDUP_SPEED  # mm/sec

        # For each leg, move toe toward the appropriate target
        any_moving = False

        # Prepare a kinematics copy to compute IK without mutating shared state prematurely
        kin = self.kinematics

        for leg_id in range(self.NUM_LEGS):
            cur_toe = kin.get_leg(leg_id).end_effector.absolute_position.copy()
            target = self.target1_toe_pos[leg_id] if stage == 1 else self.target2_toe_pos[leg_id]

            vec = target - cur_toe
            dist = np.linalg.norm(vec)
            if dist <= 1e-3:
                # already at target
                continue

            any_moving = True
            # step toward target
            step = speed * dt
            if step >= dist:
                next_pos = target
            else:
                next_pos = cur_toe + (vec / dist) * step

            # Solve IK for this leg to place toe at next_pos
            ret = kin.solve_ik_for_leg(leg_id, next_pos)
            if ret != 0:
                print(f'Warning: IK failed for leg {leg_id} moving to {next_pos}')
                continue

        # After updating IK for all legs, compute servo values and send to controller
        send_list = [7500] * self.TOTAL_SERVOS
        for sw_sid, conf in SERVO_CONFIG.items():
            leg_id = sw_sid // 3
            joint_index = sw_sid % 3
            leg = kin.get_leg(leg_id)
            if leg is None:
                continue

            angle = None
            if joint_index == 0:
                angle = leg.link3_servo.ik_angle_deg
            elif joint_index == 1:
                angle = leg.link2_servo.ik_angle_deg
            else:
                angle = leg.link1_servo.ik_angle_deg

            try:
                conv = getattr(self.controller, 'converters')[sw_sid]
                val = conv.convert_to_value(angle)
            except Exception:
                val = 7500

            phys = conf.get('physical_sid')
            if 1 <= phys <= self.TOTAL_SERVOS:
                send_list[phys - 1] = int(val)

        # Send positions
        ok, msg = self.controller.set_all_pos(send_list)
        if not ok:
            print(f'Standup: failed to send positions: {msg}')

        # Stage complete when no leg is moving
        return not any_moving
