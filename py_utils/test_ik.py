import numpy as np
import sys
import os

# Add current directory to sys.path so imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from kumokun_kinematics import KumokunKinematics
from kumokun_config import KINEMATICS_CONFIG

def test_ik_with_body_transform(body_pos, body_rot, local_target_pos):
    """
    Test IK by setting body position/rotation.
    :param body_pos: [x, y, z] body center position offset
    :param body_rot: [roll, pitch, yaw] body rotation in degrees
    :param local_target_pos: target position in leg-local frame
    """
    print(f"\n{'='*70}")
    print(f"Body Position: [{body_pos[0]:.1f}, {body_pos[1]:.1f}, {body_pos[2]:.1f}] mm")
    print(f"Body Rotation: [{body_rot[0]:.1f}, {body_rot[1]:.1f}, {body_rot[2]:.1f}] deg")
    print(f"Local Target Position: [{local_target_pos[0]:.1f}, {local_target_pos[1]:.1f}, {local_target_pos[2]:.1f}] mm")
    print(f"{'='*70}")
    
    # Initialize kinematics engine
    kinematics = KumokunKinematics()
    
    # Set body's position and rotation
    kinematics.body_center_pos = np.array(body_pos)
    kinematics.body_rotation_deg = np.array(body_rot)
    
    success_count = 0
    
    for leg_id in range(6):
        leg = kinematics.get_leg(leg_id)
        leg_mount_deg = leg.mount_angle_deg
        
        # Transform leg-local coordinates to world coordinates
        mat_leg = KumokunKinematics._create_rotation_matrix(0, 0, np.deg2rad(leg_mount_deg))
        world_target_pos = KumokunKinematics._transform_point(mat_leg, local_target_pos)
        
        # Inverse kinematics calculation (IK)
        ret = kinematics.solve_ik_for_leg(leg_id, world_target_pos)
        
        if ret == 0:
            success_count += 1
            # Retrieve computed angles
            link1_deg = leg.link1_servo.ik_angle_deg
            link2_deg = leg.link2_servo.ik_angle_deg
            link3_deg = leg.link3_servo.ik_angle_deg
            
            # Sanity check: verify coordinates using forward kinematics (FK)
            fk_pos = leg.end_effector.absolute_position
            error = np.linalg.norm(world_target_pos - fk_pos)
            
            print(f"Leg {leg_id}: OK | Angles: [{link1_deg:6.2f}, {link2_deg:6.2f}, {link3_deg:6.2f}] | Error: {error:.4f} mm")
        else:
            print(f"Leg {leg_id}: FAILED (unreachable)")
    
    print(f"\nResult: {success_count}/6 legs succeeded")
    return success_count == 6

def main():
    print("=" * 70)
    print("Inverse Kinematics Sample with Body Position/Rotation")
    print("=" * 70)
    
    # Reference target position in leg-local coordinates
    local_target_pos = np.array([200.0, 0.0, -50.0])
    
    # Test Case 1: Neutral body position (no rotation)
    print("\n[Test Case 1] Neutral Body Position and Rotation")
    test_ik_with_body_transform(
        body_pos=[0.0, 0.0, 0.0],
        body_rot=[0.0, 0.0, 0.0],
        local_target_pos=local_target_pos
    )
    
    # Test Case 2: Body moved forward
    print("\n[Test Case 2] Body Moved Forward (+X)")
    test_ik_with_body_transform(
        body_pos=[30.0, 0.0, 0.0],
        body_rot=[0.0, 0.0, 0.0],
        local_target_pos=local_target_pos
    )
    
    # Test Case 3: Body moved up
    print("\n[Test Case 3] Body Moved Up (+Z)")
    test_ik_with_body_transform(
        body_pos=[0.0, 0.0, 20.0],
        body_rot=[0.0, 0.0, 0.0],
        local_target_pos=local_target_pos
    )
    
    # Test Case 4: Body roll rotation
    print("\n[Test Case 4] Body Roll Rotation")
    test_ik_with_body_transform(
        body_pos=[0.0, 0.0, 0.0],
        body_rot=[10.0, 0.0, 0.0],
        local_target_pos=local_target_pos
    )
    
    # Test Case 5: Body pitch rotation
    print("\n[Test Case 5] Body Pitch Rotation")
    test_ik_with_body_transform(
        body_pos=[0.0, 0.0, 0.0],
        body_rot=[0.0, 10.0, 0.0],
        local_target_pos=local_target_pos
    )
    
    # Test Case 6: Body yaw rotation
    print("\n[Test Case 6] Body Yaw Rotation")
    test_ik_with_body_transform(
        body_pos=[0.0, 0.0, 0.0],
        body_rot=[0.0, 0.0, 15.0],
        local_target_pos=local_target_pos
    )
    
    # Test Case 7: Combined position and rotation
    print("\n[Test Case 7] Combined Position and Rotation")
    test_ik_with_body_transform(
        body_pos=[20.0, 10.0, 15.0],
        body_rot=[5.0, -5.0, 10.0],
        local_target_pos=local_target_pos
    )
    
    print("\n" + "=" * 70)
    print("All tests completed!")
    print("=" * 70)

if __name__ == "__main__":
    main()