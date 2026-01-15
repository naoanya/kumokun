import argparse
import sys
from servo_controller import ServoController

def main():
    # Setup command-line arguments
    parser = argparse.ArgumentParser(description='Send FREE command to servos ID 1 to 18.')
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
    
    try:
        print("Sending AF (All Free) command...")
        responses, err = controller.free_all()
        
        if err:
            print(f"Error: {err}")
        else:
            print("OK. Responses:")
            for resp in responses:
                print(f"  {resp}")
                
    finally:
        controller.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()