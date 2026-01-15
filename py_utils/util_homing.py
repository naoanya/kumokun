import argparse
import sys
import time
import json
import os
import numpy as np

# Add current directory to sys.path so imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from servo_controller import ServoController
from servo_converter import ServoConverter
from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG, HOME_POSITION, HOMING_DURATION

def main():
    # Setup command-line arguments
    parser = argparse.ArgumentParser(description='Move all legs to home position smoothly.')
    parser.add_argument('port', type=str, help='Serial port (e.g., COM3, /dev/ttyACM0)')
    parser.add_argument('--time', type=float, default=HOMING_DURATION, help=f'Duration of movement in seconds (default: {HOMING_DURATION})')
    
    args = parser.parse_args()
    
    # Instantiate the controller
    controller = ServoController()
    
    # Connect to the specified port
    print(f"Connecting to {args.port}...")
    success, msg = controller.connect(args.port)
    
    if not success:
        print(f"Failed to connect: {msg}")
        sys.exit(1)
        
    print("Connected successfully.")
    
    try:
        # 1. Release servos with AF command & read current positions
        print("Releasing servos and reading current positions...")
        responses, err = controller.free_all()
        
        if err:
            print(f"Error getting positions: {err}")
            # If error prevents continuing, exit
            return

        # Dictionary of current positions {id: value}
        current_positions = {}
        for resp in responses:
            try:
                data = json.loads(resp)
                sid = data.get("id")
                # Use pos or feedback
                pos = data.get("pos", data.get("feedback"))
                if sid is not None and pos is not None:
                    current_positions[sid] = pos
            except json.JSONDecodeError:
                pass
        
        print(f"Read positions for {len(current_positions)} servos.")

        # 2. Calculate target positions (home)
        kinematics = KumokunKinematics()
        target_positions = {} # {id: value}

        # Home position settings (mm)
        HOME_X = HOME_POSITION["x"]
        HOME_Y = HOME_POSITION["y"]
        HOME_Z = HOME_POSITION["z"]

        print(f"Calculating target positions (Home: X={HOME_X}, Y={HOME_Y}, Z={HOME_Z})...")

        converters = {}
        for sid, conf in SERVO_CONFIG.items():
            converters[sid] = ServoConverter(
                conf["direction"],
                conf["offset"],
                min_angle=conf.get("min_angle"),
                max_angle=conf.get("max_angle"),
            )
        expected_sids = set(range(6 * 3))
        missing = sorted(expected_sids - set(converters.keys()))
        if missing:
            print(f"Missing SERVO_CONFIG for sid(s): {missing}")
            return

        for leg_id in range(6):
            leg = kinematics.get_leg(leg_id)
            
            # Local -> absolute coordinate transform
            # Rotate by leg mount angle (degrees)
            mat = KumokunKinematics._create_rotation_matrix(0, 0, np.deg2rad(leg.mount_angle_deg))
            local_pos = np.array([HOME_X, HOME_Y, HOME_Z])
            abs_pos = KumokunKinematics._transform_point(mat, local_pos)
            
            # IK calculation
            ret = kinematics.solve_ik_for_leg(leg_id, abs_pos)
            if ret != 0:
                print(f"Warning: IK failed for Leg {leg_id}")
                # On IK failure, keep current position (do not move)
                continue
                
            # Convert calculated angles to servo values and store
            # Software sids: Link3=base+0, Link2=base+1, Link1=base+2
            base_sid = leg_id * 3

            sid = base_sid + 0
            conf = SERVO_CONFIG[sid]
            try:
                target_positions[conf["physical_sid"]] = converters[sid].convert_to_value(leg.link3_servo.ik_angle_deg)
            except Exception as e:
                print(f"Servo conversion error for sid {sid}: {e}")
            
            sid = base_sid + 1
            conf = SERVO_CONFIG[sid]
            try:
                target_positions[conf["physical_sid"]] = converters[sid].convert_to_value(leg.link2_servo.ik_angle_deg)
            except Exception as e:
                print(f"Servo conversion error for sid {sid}: {e}")
            
            sid = base_sid + 2
            conf = SERVO_CONFIG[sid]
            try:
                target_positions[conf["physical_sid"]] = converters[sid].convert_to_value(leg.link1_servo.ik_angle_deg)
            except Exception as e:
                print(f"Servo conversion error for sid {sid}: {e}")

        # 3. Interpolated movement
        duration = args.time
        steps = int(duration * 20) # 20Hz (50ms)
        if steps < 1: steps = 1
        dt = duration / steps
        
        print(f"Moving to home position in {duration} seconds...")
        start_time = time.time()
        
        while True:
            elapsed = time.time() - start_time
            progress = elapsed / duration
            if progress > 1.0:
                progress = 1.0
            
            send_values = []
            # Create list in order ID 1-18 (AS command spec)
            for sid in range(1, 19):
                # Start value: if not available, use neutral (7500)
                start_val = current_positions.get(sid, 7500)
                # Target value: if not calculated, use start value (no movement)
                target_val = target_positions.get(sid, start_val)

                # Linear interpolation
                curr_val = int(start_val + (target_val - start_val) * progress)
                send_values.append(curr_val)
            
            # Send
            controller.set_all_pos(send_values)
            
            if progress >= 1.0:
                break
                
            time.sleep(dt)
            
        print("Movement complete.")
                
    finally:
        controller.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()