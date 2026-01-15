import argparse
import sys
import json
import os

# Add current directory to sys.path so imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from servo_controller import ServoController
from servo_converter import ServoConverter
from kumokun_kinematics import KumokunKinematics
from kumokun_config import SERVO_CONFIG

def main():
    # Setup command-line arguments
    parser = argparse.ArgumentParser(description='Read servo positions and calculate FK.')
    parser.add_argument('port', type=str, help='Serial port (e.g., COM3, /dev/ttyACM0)')
    
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
    
    # Initialize kinematics engine
    kinematics = KumokunKinematics()

    converters = {}
    for sid, conf in SERVO_CONFIG.items():
        converters[sid] = ServoConverter(direction=conf["direction"], offset=conf["offset"])
    
    try:
        print("\nReading servo positions...")
        # For each servo (IDs 1-18), free and read position
        for sid, conf in SERVO_CONFIG.items():
            servo_id = conf["physical_sid"]
                
            # Send FREE command (release torque and read current position)
            response, err = controller.free(servo_id)
            
            #print(f"ID {servo_id}: {response}")

            if err:
                print(f"ID {servo_id}: Error: {err}")
                continue
            
            # Parse response (expecting JSON)
            try:
                # Example response: {"status":"OK","command":"FREE","id":1,"pos":7500,"feedback":7500}
                data = json.loads(response)
                # Use pos or feedback
                val = data.get("pos", data.get("feedback", 0))
            except (json.JSONDecodeError, AttributeError):
                print(f"ID {servo_id}: Parse Error. Response: {response}")
                continue
                
            # Convert to angle
            angle = converters[sid].convert_to_degrees(val)
            
            print(f"ID {servo_id} ({conf['name']}): Val={val}, Angle={angle:.2f} deg")
            
            # Apply to Kinematics model (map to current API)
            # ID 1-3 -> Leg 0, ID 4-6 -> Leg 1 ...
            leg_id = sid // 3
            leg = kinematics.get_leg(leg_id)

            # Based on order in kumokun_config.py: 1: Knee, 2: HipY, 3: HipZ
            mod_id = sid % 3
            if mod_id == 0: # Knee -> link3
                leg.link3_servo.ik_angle_deg = angle
            elif mod_id == 1: # HipY -> link2
                leg.link2_servo.ik_angle_deg = angle
            elif mod_id == 2: # HipZ -> link1
                leg.link1_servo.ik_angle_deg = angle
                
        # Forward Kinematics calculation
        print("\nCalculating Forward Kinematics...")
        kinematics.forward_kinematics_all()
        
        for i in range(6):
            leg = kinematics.get_leg(i)
            pos = leg.end_effector.absolute_position
            print(f"Leg {i}: Toe Pos = [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
                
    finally:
        controller.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()