"""
Kumo-kun Kinematics Library
Handles forward and inverse kinematics calculations for the hexapod.

=============================================================================
ROBOT STRUCTURE & COORDINATE SYSTEMS
=============================================================================

[1] SINGLE LEG STRUCTURE (Link Chain)
====================================

    Body Center ---> (Y+ for Leg 0)
        |
        | (origin_to_link1)
        |
    Link1 (Coxa Joint)
        |
        | (link1_to_link2, rotated by link1 angle)
        |
    Link2 (Femur Joint)
        |
        | (link2_to_link3, rotated by link2 angle)
        |
    Link3 (Tibia Joint)
        |
        | (link3_to_toe, rotated by link3 angle)
        |
    Toe (End Effector)
        |
        v (X+ for Leg 0)

    Each leg has 3 joints with 4 link segments:
    - origin_to_link1: Distance from body center to Link1 joint
    - link1_to_link2: Distance from Link1 joint to Link2 joint
    - link2_to_link3: Distance from Link2 joint to Link3 joint
    - link3_to_toe: Distance from Link3 joint to Toe (end effector)

    Default Posture (All servo angles at 0 degrees):
    - The leg is fully extended in a straight line
    - For Leg 0 in default posture: direction from Body Center to Toe is along the X+ axis
    - Coordinate system axes:
      * X+ axis: Forward direction (Body Center to Toe for Leg 0)
      * Y+ axis: Left direction
      * Z+ axis: Upward direction
    - Joint rotation axes in this coordinate system for Leg 0:
      * Link1: Rotates around Z-axis (vertical rotation, leg sweep)
      * Link2: Rotates around Y-axis (pitch rotation, leg lift)
      * Link3: Rotates around Y-axis (pitch rotation, knee bend)

[2] HEXAPOD LEG ARRANGEMENT
===========================

         Top View (looking down at XY plane)
         
                 ^ X+
                 :
               Leg 0
                 |
          Leg 1  |  Leg 5
               i | /
                i|/
    Y+ <... (Body Center)
                /|i
               / | i
          Leg 2  |  Leg 4
                 |
               Leg 3

    Mount Angles (degrees from X-axis):
    - Leg 0: 0°    (Front direction)
    - Leg 1: 60°   (Front-Left direction)
    - Leg 2: 120°  (Back-Left direction)
    - Leg 3: 180°  (Back direction)
    - Leg 4: 240°  (Back-Right direction)
    - Leg 5: 300°  (Front-Right direction)

[3] COORDINATE SYSTEMS
=======================

A) WORLD COORDINATES (Fixed Global Frame)
   -----------------------------------------
   - Origin: Robot center (where all legs converge)
   - X-axis: Forward direction
   - Y-axis: Left direction
   - Z-axis: Up direction
   - Properties: Fixed in space, unaffected by robot motion
   - Usage: Input target positions for IK calculation
   
B) BODY COORDINATES (Local Robot Frame)
   ------------------------------------
   - Origin: Body center position (body_center_pos)
   - Offset: body_center_pos + [0, 0, default_height] in Z
   - Rotation: body_rotation_deg applied (Roll, Pitch, Yaw)
   - Properties: Moves with the robot body
   - Usage: Internal IK calculation frame
   
C) LEG COORDINATES (Local Leg Frame)
   --------------------------------
   - Origin: Body center (same as body frame origin)
   - Rotation: leg.mount_angle_deg (rotation around Z-axis only)
   - Properties: Pre-rotated to point along each leg's direction
   - Usage: Convenient frame for defining symmetric leg positions

[4] COORDINATE TRANSFORMATION FLOW (World → IK Solver)
======================================================

    User Input:
    - body_center_pos [x, y, z]
    - body_rotation_deg [roll, pitch, yaw]
    - target_toe_pos (in WORLD coordinates)

    Step 1: Set body configuration
            kinematics.body_center_pos = [x, y, z]
            kinematics.body_rotation_deg = [roll, pitch, yaw]

    Step 2: Create target position in leg-local frame
            (for convenience, define symmetric positions for all legs)
            local_target_pos = [200, 0, -50]

    Step 3: Transform to world coordinates
            mat_leg = rotation_matrix(0, 0, leg.mount_angle_deg)
            world_target_pos = mat_leg @ local_target_pos

    Step 4: Pass to IK solver (automatically transforms to body frame)
            kinematics.solve_ik_for_leg(leg_id, world_target_pos)
            
            Internal process in solve_ik_for_leg():
            a) Compute effective body position:
               effective_body_pos = body_center_pos + [0, 0, default_height]
            b) Transform world coordinates to body frame:
               target_in_body_frame = inverse_body_rotation @ (world_target_pos - effective_body_pos)
            c) Calculate Link1 angle using leg_mount_deg (body frame XY plane)
            d) Transform target to leg frame (remove leg_mount_deg and Link1 position)
            e) Solve Link2, Link3 angles using Law of Cosines (leg frame 2D triangle)

[5] EXAMPLE: SETTING LEG POSITIONS
===================================

    # Initialize
    kinematics = KumokunKinematics()
    
    # Configure robot body
    kinematics.body_center_pos = [0, 0, 0]          # No offset
    kinematics.body_rotation_deg = [0, 0, 0]        # No rotation
    
    # Define target in leg-local frame (convenient for symmetric positions)
    local_target = [200.0, 0.0, -50.0]
    
    # For each leg, calculate world target position
    for leg_id in range(6):
        leg = kinematics.get_leg(leg_id)
        
        # Transform from leg-local to world using leg mount angle
        mat = KumokunKinematics._create_rotation_matrix(0, 0, np.deg2rad(leg.mount_angle_deg))
        world_target = mat @ local_target
        
        # Solve IK
        kinematics.solve_ik_for_leg(leg_id, world_target)

=============================================================================
"""

import numpy as np
import math
from dataclasses import dataclass, field
from kumokun_config import KINEMATICS_CONFIG



# --- Data Classes ---

@dataclass
class Servo:
    """
    Represents a single servo motor with its kinematic state.
    
    Attributes:
    - ik_angle_deg: Joint angle in degrees from the IK calculation
    - body_relative_position: Position in BODY FRAME coordinates
      * Origin: Body center (body_center_pos with default_height)
      * Rotation: Mount angle applied (but NOT body rotation)
      * Usage: Position of servo relative to body frame, before body rotation
    - absolute_position: Position in WORLD COORDINATES (absolute, fixed reference frame)
      * Origin: World origin (0, 0, 0)
      * Rotation: Both mount angle AND body rotation applied
      * Usage: Actual world position of the servo
    - absolute_rotation_deg: Rotation in WORLD COORDINATES [roll, pitch, yaw] in degrees
    """
    ik_angle_deg: float = 0.0
    absolute_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    body_relative_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    absolute_rotation_deg: np.ndarray = field(default_factory=lambda: np.zeros(3))

@dataclass
class Leg:
    """Represents a single leg of the robot, containing its servos."""
    id: int
    mount_angle_deg: float
    link1_servo: Servo = field(default_factory=Servo)
    link2_servo: Servo = field(default_factory=Servo)
    link3_servo: Servo = field(default_factory=Servo)
    end_effector: Servo = field(default_factory=Servo)  # Virtual servo for toe position

@dataclass
class RobotGeometry:
    """Stores the fixed link lengths of the robot."""
    origin_to_link1: np.ndarray
    link1_to_link2: np.ndarray
    link2_to_link3: np.ndarray
    link3_to_toe: np.ndarray

@dataclass
class ForwardKinematicsResult:
    """Result of forward kinematics calculation for a single leg."""
    success: int  # 0 on success
    # Body-relative positions (mount angle applied, body rotation NOT applied)
    link1_body_rel: np.ndarray
    link2_body_rel: np.ndarray
    link3_body_rel: np.ndarray
    toe_body_rel: np.ndarray
    # Absolute world positions (mount angle AND body rotation applied)
    link1_abs: np.ndarray
    link2_abs: np.ndarray
    link3_abs: np.ndarray
    toe_abs: np.ndarray
    # Absolute rotations in world frame [roll, pitch, yaw]
    rot_link1_abs: np.ndarray
    rot_link2_abs: np.ndarray
    rot_link3_abs: np.ndarray


# --- Main Kinematics Class ---

class KumokunKinematics:
    """
    Kumo-kun Kinematics Library.
    Handles forward and inverse kinematics calculations for the hexapod.
    """
    # Constants
    NUM_LEGS = 6
    DEGREES_PER_LEG = 360.0 / NUM_LEGS
    XY_PLANE_Z_INDEX = 2
    EPSILON = 1e-9
    CLAMPING_RANGE = (-1.0, 1.0)
    
    # Angle constants (degrees)
    PERPENDICULAR_ANGLE_DEG = 90.0
    STRAIGHT_ANGLE_DEG = 180.0
    
    # Unit vectors
    UNIT_VECTOR_Z = np.array([0.0, 0.0, 1.0])  # Vertical/upward direction
    
    def __init__(self, config: dict | None = None) -> None:
        """
        Initializes the kinematics engine with robot configuration.
        :param config: A dictionary overriding default KINEMATICS_CONFIG values.
        """
        _config = KINEMATICS_CONFIG.copy()
        if config:
            _config.update(config)

        self.default_body_height = _config["lBodyHeight"]

        # --- Robot State ---
        # These are part of the public API to be set by the controller.
        self.body_center_pos: np.ndarray = np.zeros(3) # X, Y, Z
        self.body_rotation_deg: np.ndarray = np.zeros(3) # Roll, Pitch, Yaw in degrees

        # --- Internal Geometric Constants ---
        self._geometry = RobotGeometry(
            origin_to_link1=np.array([_config["lOriginToLink1"], 0.0, 0.0]),
            link1_to_link2=np.array([_config["lXLink1ToLink2"], 0.0, _config["lZLink1ToLink2"]]),
            link2_to_link3=np.array([_config["lLink2ToLink3"], 0.0, 0.0]),
            link3_to_toe=np.array([_config["lLink3ToToe"], 0.0, 0.0])
        )

        # --- Robot Structure ---
        self.legs: list[Leg] = [
            Leg(i, self.DEGREES_PER_LEG * i) for i in range(self.NUM_LEGS)
        ]
        
        # Initialize all leg positions based on default angles (0)
        self.forward_kinematics_all()

    def get_leg(self, leg_id: int) -> Leg | None:
        """
        Gets the Leg object by its ID (index).
        :param leg_id: The ID of the leg (0-5).
        :return: The Leg object or None if the ID is invalid.
        """
        if 0 <= leg_id < self.NUM_LEGS:
            return self.legs[leg_id]
        return None

    def solve_ik_for_leg(self, leg_id: int, target_toe_pos: np.ndarray, do_fk: bool = True) -> int:
        """
        Calculates inverse kinematics for a single leg.
        Updates the `ik_angle_deg` attributes of the leg's servos.
        
        Coordinate System:
        - target_toe_pos is the target position in WORLD COORDINATES (absolute, fixed reference frame)
        - The kinematics solver internally:
          1. Converts world coordinates to body frame using body_center_pos 
             (with default_height added) and body_rotation_deg
          2. Solves IK to position the toe at the target world coordinate
        - Guarantee: The toe will be positioned at target_toe_pos regardless of changes to
          body_center_pos, body_rotation_deg, or default_height
        
        Usage Example:
            kinematics = KumokunKinematics()
            kinematics.body_center_pos = np.array([x, y, z])
            kinematics.body_rotation_deg = np.array([roll, pitch, yaw])
            
            # Define target position using Leg 0's local coordinate system
            # (this is a convenient way to define the same target for all legs)
            local_target_pos = np.array([200.0, 0.0, -50.0])
            
            # For each leg, transform to world coordinates by applying that leg's mount angle
            leg = kinematics.get_leg(leg_id)
            leg_mount_rad = math.radians(leg.mount_angle_deg)
            rot_matrix = KumokunKinematics._create_rotation_matrix(0, 0, leg_mount_rad)
            world_target_pos = rot_matrix @ local_target_pos
            
            # Solve IK with absolute world position
            ret = kinematics.solve_ik_for_leg(leg_id, world_target_pos)
        
        :param leg_id: The ID of the leg to calculate for (0-5)
        :param target_toe_pos: The target absolute world position for the toe (x, y, z) in mm
        :param do_fk: If True, runs forward kinematics for the leg after IK to update coordinates
        :return: 0 on success, -1 on failure (e.g., unreachable)
        """
        leg = self.get_leg(leg_id)
        if leg is None:
            return -1

        effective_body_pos = self._compute_effective_body_position()

        ret, link1_angle_deg, link2_angle_deg, link3_angle_deg = self._calculate_ik(
            geometry=self._geometry,
            body_origin=effective_body_pos,
            body_rot_deg=self.body_rotation_deg,
            leg_mount_deg=leg.mount_angle_deg,
            target_toe_pos=target_toe_pos,
            current_link1_angle_deg=leg.link1_servo.ik_angle_deg
        )

        if ret != 0:
            return ret # Calculation failed

        # Update leg servo angles
        leg.link1_servo.ik_angle_deg = link1_angle_deg
        leg.link2_servo.ik_angle_deg = link2_angle_deg
        leg.link3_servo.ik_angle_deg = link3_angle_deg

        if do_fk:
            self.solve_fk_for_leg(leg)
        
        return 0

    def forward_kinematics_all(self) -> int:
        """Calculates forward kinematics for all legs."""
        for leg in self.legs:
            self.solve_fk_for_leg(leg)
        return 0

    def solve_fk_for_leg(self, leg: Leg) -> int:
        """
        Calculates forward kinematics for a single leg based on its `ik_angle_deg` values.
        Updates the position and rotation attributes of the leg's servos.
        :param leg: The Leg object to update.
        :return: 0 on success.
        """
        effective_body_pos = self._compute_effective_body_position()

        fk_results = self._calculate_fk(
            geometry=self._geometry,
            body_origin=effective_body_pos,
            body_rot_deg=self.body_rotation_deg,
            leg_mount_deg=leg.mount_angle_deg,
            link1_angle_deg=leg.link1_servo.ik_angle_deg,
            link2_angle_deg=leg.link2_servo.ik_angle_deg,
            link3_angle_deg=leg.link3_servo.ik_angle_deg
        )

        self._update_leg_from_fk_results(leg, fk_results)
        return 0

    # --- Helper Methods ---
    
    def _compute_effective_body_position(self) -> np.ndarray:
        """Computes the effective body position by adding default height."""
        return self.body_center_pos + np.array([0.0, 0.0, self.default_body_height])
    
    def _update_leg_from_fk_results(self, leg: Leg, fk_result: ForwardKinematicsResult) -> None:
        """Updates leg servo positions and rotations from FK calculation results."""
        leg.link1_servo.body_relative_position = fk_result.link1_body_rel
        leg.link2_servo.body_relative_position = fk_result.link2_body_rel
        leg.link3_servo.body_relative_position = fk_result.link3_body_rel
        leg.end_effector.body_relative_position = fk_result.toe_body_rel
        
        leg.link1_servo.absolute_position = fk_result.link1_abs
        leg.link2_servo.absolute_position = fk_result.link2_abs
        leg.link3_servo.absolute_position = fk_result.link3_abs
        leg.end_effector.absolute_position = fk_result.toe_abs
        
        leg.link1_servo.absolute_rotation_deg = fk_result.rot_link1_abs
        leg.link2_servo.absolute_rotation_deg = fk_result.rot_link2_abs
        leg.link3_servo.absolute_rotation_deg = fk_result.rot_link3_abs
    
    # --- Static Calculation Methods (Internal Logic) ---
    # These methods contain the core mathematical logic and are kept pure.

    @staticmethod
    def _calculate_law_of_cosines_angle(side_a: float, side_b: float, side_c: float) -> float:
        """Calculates angle B using Law of Cosines: b² = a² + c² - 2ac·cos(B)."""
        if side_a + side_c == side_b:
            return math.pi
        val = ((side_c * side_c) + (side_a * side_a) - (side_b * side_b)) / (2 * side_c * side_a)
        val = max(KumokunKinematics.CLAMPING_RANGE[0], min(KumokunKinematics.CLAMPING_RANGE[1], val))
        return math.acos(val)
    
    @staticmethod
    def _is_target_reachable(distance: float, link_a: float, link_b: float) -> bool:
        """
        Checks if target is reachable by two-link chain.
        
        :param distance: Distance from origin to target
        :param link_a: Length of first link
        :param link_b: Length of second link
        :return: True if reachable, False otherwise
        """
        max_reach = link_a + link_b
        min_reach = abs(link_a - link_b)
        return min_reach <= distance <= max_reach
    
    @staticmethod
    def _calculate_link1_angle(
        geometry: RobotGeometry,
        target_body_frame: np.ndarray,
        leg_mount_deg: float,
        current_angle_deg: float
    ) -> float:
        """
        Calculates Link1 (coxa) joint angle in body frame.
        
        :param geometry: Robot geometry
        :param target_body_frame: Target position in body frame
        :param leg_mount_deg: Leg mount angle in degrees
        :param current_angle_deg: Current Link1 angle (fallback if target is on axis)
        :return: Link1 angle in degrees
        """
        # Vector from body center to Link1 joint (rotated by mount angle)
        link1_pos = KumokunKinematics._rotate_z_deg(geometry.origin_to_link1, leg_mount_deg)
        
        # Project vectors to XY plane
        body_to_link1 = link1_pos.copy()
        link1_to_target = target_body_frame - link1_pos
        body_to_link1[KumokunKinematics.XY_PLANE_Z_INDEX] = 0
        link1_to_target[KumokunKinematics.XY_PLANE_Z_INDEX] = 0

        if np.allclose(link1_to_target, 0):
            # Target is on Link1 axis, keep current angle
            return current_angle_deg
        
        # Calculate angle between vectors
        vec1_norm = KumokunKinematics._normalize_vector(body_to_link1)
        vec2_norm = KumokunKinematics._normalize_vector(link1_to_target)
        dot_product = np.clip(np.dot(vec1_norm, vec2_norm), -1.0, 1.0)
        
        angle_rad = math.acos(dot_product)
        angle_deg = math.degrees(angle_rad)
        
        # Determine rotation direction using cross product
        direction = KumokunKinematics._get_cross_product_direction_z(body_to_link1, link1_to_target)
        return direction * angle_deg
    
    @staticmethod
    def _calculate_link2_link3_angles(
        toe_pos_leg_frame: np.ndarray,
        link2_to_link3_dist: float,
        link3_to_toe_dist: float
    ) -> tuple[float, float]:
        """
        Calculates Link2 (femur) and Link3 (tibia) joint angles.
        
        :param toe_pos_leg_frame: Toe position in leg frame
        :param link2_to_link3_dist: Distance from Link2 to Link3 joint
        :param link3_to_toe_dist: Distance from Link3 to toe
        :return: (link2_angle_deg, link3_angle_deg)
        """
        link2_to_toe_dist = np.linalg.norm(toe_pos_leg_frame)
        
        # Use Law of Cosines to find triangle angles
        angle_at_link3_rad = KumokunKinematics._calculate_law_of_cosines_angle(
            link2_to_link3_dist, link3_to_toe_dist, link2_to_toe_dist
        )
        angle_at_toe_rad = KumokunKinematics._calculate_law_of_cosines_angle(
            link2_to_link3_dist, link2_to_toe_dist, link3_to_toe_dist
        )
        
        angle_at_link3_deg = math.degrees(angle_at_link3_rad)
        angle_at_toe_deg = math.degrees(angle_at_toe_rad)

        # Calculate Link2 orientation
        link2_oriented = KumokunKinematics._calculate_link2_oriented(
            toe_pos_leg_frame, angle_at_link3_deg, link2_to_link3_dist
        )

        # Calculate angle between vertical and Link2
        vertical_to_link2_dist = np.linalg.norm(link2_oriented - KumokunKinematics.UNIT_VECTOR_Z)
        link2_length = np.linalg.norm(link2_oriented)
        
        angle_vertical_to_link2_rad = KumokunKinematics._calculate_law_of_cosines_angle(
            1.0, vertical_to_link2_dist, link2_length
        )
        angle_vertical_to_link2_deg = math.degrees(angle_vertical_to_link2_rad)
        
        if link2_oriented[0] < 0:
            angle_vertical_to_link2_deg = -angle_vertical_to_link2_deg

        link2_angle_deg = -(KumokunKinematics.PERPENDICULAR_ANGLE_DEG - angle_vertical_to_link2_deg)
        link3_angle_deg = KumokunKinematics.STRAIGHT_ANGLE_DEG - angle_at_toe_deg

        return link2_angle_deg, link3_angle_deg
    
    @staticmethod
    def _calculate_ik(
        geometry: RobotGeometry,
        body_origin: np.ndarray,
        body_rot_deg: np.ndarray,
        leg_mount_deg: float,
        target_toe_pos: np.ndarray,
        current_link1_angle_deg: float
    ) -> tuple[int, float, float, float]:
        """
        Calculates inverse kinematics for a single leg.
        
        IK Calculation Flow:
        ====================
        1. Transform target position from WORLD coordinates to BODY FRAME coordinates
           - Removes body_rotation_deg and body_origin offset
           - Result: target position relative to body center, unrotated
        
        2. Calculate Link1 angle in BODY FRAME (XY plane)
           - leg_mount_deg is applied here to position Link1 joint correctly
           - Link1 rotates around the vertical (Z) axis in body frame
           - Determines the direction from body center to the target toe
        
        3. Transform target position from BODY FRAME to LEG FRAME
           - Remove leg_mount_deg rotation (undo the leg's mounting angle)
           - Remove Link1 joint position and Link1 angle effects
           - Result: target position relative to Link2 joint, in leg's local coordinate system
        
        4. Calculate Link2 and Link3 angles in LEG FRAME (2D triangle)
           - Solves 2D triangle geometry: Link2 joint → Link3 joint → Toe
           - Uses Law of Cosines to find joint angles
        
        COORDINATE FRAME SEQUENCE:
        - WORLD coords (input) → BODY frame (step 1) → LEG frame (step 3) → Joint angles (step 4)
        
        LEG MOUNT ANGLE ROLE:
        - Positions each leg's origin_to_link1 distance around the body center
        - Applied as Z-axis rotation in body frame
        - Transforms body-frame geometry to be aligned with each leg's local coordinate system
        """

        # Step 1: Transform to body frame
        target_body_frame = KumokunKinematics._world_to_body_frame(
            target_toe_pos, body_origin, body_rot_deg
        )

        # Step 2: Calculate Link1 angle
        link1_angle_deg = KumokunKinematics._calculate_link1_angle(
            geometry, target_body_frame, leg_mount_deg, current_link1_angle_deg
        )

        # Step 3: Transform to leg frame
        toe_pos_leg_frame = KumokunKinematics._transform_to_leg_frame(
            target_body_frame, leg_mount_deg, link1_angle_deg, geometry
        )

        # Step 4: Check reachability and calculate Link2, Link3 angles
        link2_to_link3_dist = np.linalg.norm(geometry.link2_to_link3)
        link3_to_toe_dist = np.linalg.norm(geometry.link3_to_toe)
        toe_distance = np.linalg.norm(toe_pos_leg_frame)
        
        if not KumokunKinematics._is_target_reachable(toe_distance, link2_to_link3_dist, link3_to_toe_dist):
            return -1, 0, 0, 0  # Target is unreachable

        link2_angle_deg, link3_angle_deg = KumokunKinematics._calculate_link2_link3_angles(
            toe_pos_leg_frame, link2_to_link3_dist, link3_to_toe_dist
        )

        # Validate results
        if math.isnan(link1_angle_deg) or math.isnan(link2_angle_deg) or math.isnan(link3_angle_deg):
            return -1, 0, 0, 0

        return 0, link1_angle_deg, link2_angle_deg, link3_angle_deg

    @staticmethod
    def _calculate_fk(
        geometry: RobotGeometry,
        body_origin: np.ndarray,
        body_rot_deg: np.ndarray,
        leg_mount_deg: float,
        link1_angle_deg: float,
        link2_angle_deg: float,
        link3_angle_deg: float
    ) -> ForwardKinematicsResult:
        """
        Calculates forward kinematics for a single leg.
        
        :return: ForwardKinematicsResult containing all servo positions and rotations
        """
        # Rotation Matrices
        rot_mount = KumokunKinematics._create_rotation_matrix(0, 0, math.radians(leg_mount_deg))
        rot_link1 = KumokunKinematics._create_rotation_matrix(0, 0, math.radians(link1_angle_deg))
        rot_link2 = KumokunKinematics._create_rotation_matrix(0, math.radians(link2_angle_deg), 0)
        rot_link3 = KumokunKinematics._create_rotation_matrix(0, math.radians(link3_angle_deg), 0)
        
        rot_body = KumokunKinematics._create_rotation_matrix(
            math.radians(body_rot_deg[0]),
            math.radians(body_rot_deg[1]),
            math.radians(body_rot_deg[2])
        )

        # Chain in Leg Frame (Relative to Body Center, unrotated mount)
        # 1. Link1
        p_link1_local = geometry.origin_to_link1
        
        # 2. Link2
        # Rotated by Link1 angle
        p_link2_local = p_link1_local + rot_link1 @ geometry.link1_to_link2
        
        # 3. Link3
        # Rotated by Link1 * Link2
        rot_to_link2 = rot_link1 @ rot_link2
        p_link3_local = p_link2_local + rot_to_link2 @ geometry.link2_to_link3
        
        # 4. Toe (End effector)
        rot_to_link3 = rot_to_link2 @ rot_link3
        p_toe_local = p_link3_local + rot_to_link3 @ geometry.link3_to_toe
        
        # Apply Mount Rotation to get Body Frame positions
        # Body frame: Origin at body_center_pos + default_height, rotated by mount angle only
        v_link1_body_rel = rot_mount @ p_link1_local
        v_link2_body_rel = rot_mount @ p_link2_local
        v_link3_body_rel = rot_mount @ p_link3_local
        v_toe_body_rel = rot_mount @ p_toe_local
        
        # Apply Body Rotation and Translation to get Absolute World positions
        # World frame: Fixed global reference, includes body rotation and position offset
        v_link1_abs = body_origin + rot_body @ v_link1_body_rel
        v_link2_abs = body_origin + rot_body @ v_link2_body_rel
        v_link3_abs = body_origin + rot_body @ v_link3_body_rel
        v_toe_abs = body_origin + rot_body @ v_toe_body_rel
        
        # Rotations (Absolute) - Simplified approximation
        rot_link1_abs = np.array([0.0, 0.0, leg_mount_deg + link1_angle_deg])
        rot_link2_abs = np.array([link2_angle_deg, 0.0, leg_mount_deg + link1_angle_deg])
        rot_link3_abs = np.array([link2_angle_deg + link3_angle_deg, 0.0, leg_mount_deg + link1_angle_deg])

        return ForwardKinematicsResult(
            success=0,
            link1_body_rel=v_link1_body_rel,
            link2_body_rel=v_link2_body_rel,
            link3_body_rel=v_link3_body_rel,
            toe_body_rel=v_toe_body_rel,
            link1_abs=v_link1_abs,
            link2_abs=v_link2_abs,
            link3_abs=v_link3_abs,
            toe_abs=v_toe_abs,
            rot_link1_abs=rot_link1_abs,
            rot_link2_abs=rot_link2_abs,
            rot_link3_abs=rot_link3_abs
        )

    # --- Matrix Helpers ---

    @staticmethod
    def _create_rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
        """
        Creates a 3x3 rotation matrix from Euler angles (ZYX convention).
        
        :param rx: Rotation around X-axis in radians
        :param ry: Rotation around Y-axis in radians
        :param rz: Rotation around Z-axis in radians
        :return: 3x3 rotation matrix
        """
        # R = Rz * Ry * Rx
        # Optimized: Use math functions and pre-calculated expansion
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)

        # Rz @ Ry @ Rx expanded (3x3)
        R = np.array([
            [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
            [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
            [-sy,   cy*sx,            cy*cx]
        ])
        return R
    
    @staticmethod
    def _create_rotation_matrix_deg(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
        """
        Creates a 3x3 rotation matrix from Euler angles in degrees.
        
        :param rx_deg: Rotation around X-axis in degrees
        :param ry_deg: Rotation around Y-axis in degrees
        :param rz_deg: Rotation around Z-axis in degrees
        :return: 3x3 rotation matrix
        """
        return KumokunKinematics._create_rotation_matrix(
            math.radians(rx_deg),
            math.radians(ry_deg),
            math.radians(rz_deg)
        )
    
    @staticmethod
    def _normalize_vector(vec: np.ndarray) -> np.ndarray:
        """
        Normalizes a vector to unit length.
        
        :param vec: Input vector
        :return: Normalized vector (unit length), or zero vector if input is near-zero
        """
        norm = np.linalg.norm(vec)
        if norm < KumokunKinematics.EPSILON:
            return np.zeros_like(vec)
        return vec / norm

    # --- Coordinate Transformation Helpers ---
    
    @staticmethod
    def _world_to_body_frame(target_pos: np.ndarray, body_origin: np.ndarray, body_rot_deg: np.ndarray) -> np.ndarray:
        """Transforms a world-frame position to the body's local coordinate frame."""
        rot_matrix = KumokunKinematics._create_rotation_matrix_deg(
            body_rot_deg[0],
            body_rot_deg[1],
            body_rot_deg[2]
        )
        vec_body_to_target = target_pos - body_origin
        return rot_matrix.T @ vec_body_to_target
    
    @staticmethod
    def _rotate_z_deg(vector: np.ndarray, angle_deg: float) -> np.ndarray:
        """Rotates a vector around Z axis by angle in degrees."""
        mat = KumokunKinematics._create_rotation_matrix(0, 0, math.radians(angle_deg))
        return mat @ vector
    
    @staticmethod
    def _transform_to_leg_frame(
        target_pos: np.ndarray, leg_mount_deg: float, link1_deg: float, geometry: RobotGeometry
    ) -> np.ndarray:
        """Transforms target position to the leg's local coordinate frame."""
        pos = target_pos.copy()
        pos = KumokunKinematics._rotate_z_deg(pos, -leg_mount_deg)
        pos = pos - geometry.origin_to_link1
        pos = KumokunKinematics._rotate_z_deg(pos, -link1_deg)
        pos = pos - geometry.link1_to_link2
        return pos
    
    @staticmethod
    def _get_cross_product_direction_z(vec_a: np.ndarray, vec_b: np.ndarray) -> float:
        """
        Returns direction (±1) based on Z component of cross product.
        
        :param vec_a: First vector
        :param vec_b: Second vector
        :return: 1.0 if Z component is positive or zero, -1.0 if negative
        """
        axis = np.cross(vec_a, vec_b)
        axis_norm = KumokunKinematics._normalize_vector(axis)
        return 1.0 if axis_norm[2] >= 0 else -1.0
    
    @staticmethod
    def _calculate_link2_oriented(
        toe_pos_leg_frame: np.ndarray, angle_deg: float, link2_to_link3_distance: float
    ) -> np.ndarray:
        """
        Calculates the link2 vector oriented in leg frame.
        
        :param toe_pos_leg_frame: Toe position in leg frame
        :param angle_deg: Angle in degrees
        :param link2_to_link3_distance: Distance from Link2 to Link3 joint
        :return: Oriented link2 vector
        """
        vec = toe_pos_leg_frame.copy()
        rot_mat = KumokunKinematics._create_rotation_matrix(0, math.radians(-angle_deg), 0)
        vec = rot_mat @ vec
        vec_norm = KumokunKinematics._normalize_vector(vec)
        return vec_norm * link2_to_link3_distance
    
    @staticmethod
    def _transform_point(matrix: np.ndarray, point: np.ndarray) -> np.ndarray:
        """Applies a 3x3 rotation matrix to a 3D point."""
        return matrix @ point