import argparse
import sys
import time
import os

# Add current directory to sys.path so imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from servo_controller import ServoController

def main():
    # Setup command-line arguments
    parser = argparse.ArgumentParser(description='Measure transfer rate with AS command (sending 0 to all servos).')
    parser.add_argument('port', type=str, help='Serial port (e.g., COM3, /dev/ttyACM0)')
    parser.add_argument('--duration', type=float, default=10.0, help='Test duration in seconds (default: 10.0)')
    
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
    print(f"Starting transfer rate test for {args.duration} seconds...")
    print("Sending AS command with position 0 (free) to all 18 servos repeatedly.")
    
    # Target positions to send to all servos (all zeros)
    # In the ICSCtrl implementation, setPosition(id, 0) is equivalent to setFree(id).
    target_positions = [0] * 18
    
    count = 0
    start_time = time.time()
    end_time = start_time + args.duration
    
    try:
        while time.time() < end_time:
            # Send AS command
            # Return value is (responses, err)
            # set_all_pos calls send_and_receive_lines internally and blocks
            # until 18 response lines are received.
            responses, err = controller.set_all_pos(target_positions)
            
            if err:
                print(f"\nError occurred at iteration {count + 1}: {err}")
                # Break the loop on error
                break
            
            # Increment success count
            count += 1
            
            # Simple progress indicator (print a dot every 10 iterations)
            if count % 10 == 0:
                print(".", end="", flush=True)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        
    finally:
        actual_end_time = time.time()
        elapsed = actual_end_time - start_time
        controller.disconnect()
        print("\nDisconnected.")

    # Display results
    print("\n" + "=" * 40)
    print(f"Test Results:")
    print(f"  Duration:       {elapsed:.3f} seconds")
    print(f"  Total Commands: {count}")
    
    if elapsed > 0:
        rate = count / elapsed
        print(f"  Transfer Rate:  {rate:.2f} Hz (commands/sec)")
        if count > 0:
            avg_latency = (elapsed / count) * 1000
            print(f"  Avg Latency:    {avg_latency:.2f} ms/command")
    else:
        print("  Transfer Rate:  N/A")
    print("=" * 40)

if __name__ == "__main__":
    main()