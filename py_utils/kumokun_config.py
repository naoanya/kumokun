"""
Kumo-kun Robot Configuration
"""

# Kinematics Link Lengths [mm]
# Link1: Origin to Link1 (hip_z equivalent)
# Link2: Link1 to Link2 (hip_y equivalent)
# Link3: Link2 to Link3 (knee equivalent)
KINEMATICS_CONFIG = {
    "lBodyHeight": 1.0,
    "lOriginToLink1": 70.0,
    "lXLink1ToLink2": 30.0,
    "lZLink1ToLink2": -1.0,
    "lLink2ToLink3": 80.0,
    "lLink3ToToe": 175.0
}

# Home Position Configuration [mm]
# Base extension from hip center, and height relative to body center
HOME_POSITION = {
    "x": 300.0,
    "y": 0.0,
    "z": 50.0
}

# Homing Duration [sec]
HOMING_DURATION = 5.0

# Standup Position Configuration [mm]
STANDUP_POSITION = {
    "x": 210.0,
    "y": 0.0,
    "z": -100.0
}

# Standup Speed [mm/sec]
STANDUP_SPEED = 20.0

# Update Interval [sec]
UPDATE_INTERVAL = 0.01

# Servo Configuration
# Software ID (sid): Software-side ID used in kinematics/control (0-17)
# Hardware ID (physical_sid): Physical servo ID used for communication (hardware-dependent; values may vary)
# sid: { "physical_sid": hardware id, "direction": 1 or -1, "offset": degree,
#        "name": string, "min_angle": degree, "max_angle": degree }
# Note: Direction 1 is Normal, -1 is Reverse
#       Offset is added to the calculated angle before conversion to servo value
#       min_angle/max_angle are used for software limit checks (in degrees)
SERVO_CONFIG = {
    # Leg 0 (Right Front)
    0: {"name": "Leg0_Link3", "physical_sid": 1, "direction":  1, "offset": -67.5,   "min_angle": -135.0, "max_angle": 135.0},
    1: {"name": "Leg0_Link2", "physical_sid": 2, "direction": -1, "offset": -43.875, "min_angle": -135.0, "max_angle": 135.0},
    2: {"name": "Leg0_Link1", "physical_sid": 3, "direction":  1, "offset":   0.0,   "min_angle": -135.0, "max_angle": 135.0},

    # Leg 1 (Right Middle)
    3: {"name": "Leg1_Link3", "physical_sid": 4, "direction":  1, "offset": -67.5,   "min_angle": -135.0, "max_angle": 135.0},
    4: {"name": "Leg1_Link2", "physical_sid": 5, "direction": -1, "offset": -43.875, "min_angle": -135.0, "max_angle": 135.0},
    5: {"name": "Leg1_Link1", "physical_sid": 6, "direction":  1, "offset":   0.0,   "min_angle": -135.0, "max_angle": 135.0},

    # Leg 2 (Right Rear)
    6: {"name": "Leg2_Link3", "physical_sid": 7, "direction":  1, "offset": -67.5,   "min_angle": -135.0, "max_angle": 135.0},
    7: {"name": "Leg2_Link2", "physical_sid": 8, "direction": -1, "offset": -43.875, "min_angle": -135.0, "max_angle": 135.0},
    8: {"name": "Leg2_Link1", "physical_sid": 9, "direction":  1, "offset":   0.0,   "min_angle": -135.0, "max_angle": 135.0},

    # Leg 3 (Left Rear)
    9:  {"name": "Leg3_Link3", "physical_sid": 10, "direction":  1, "offset": -67.5,   "min_angle": -135.0, "max_angle": 135.0},
    10: {"name": "Leg3_Link2", "physical_sid": 11, "direction": -1, "offset": -43.875, "min_angle": -135.0, "max_angle": 135.0},
    11: {"name": "Leg3_Link1", "physical_sid": 12, "direction":  1, "offset":   0.0,   "min_angle": -135.0, "max_angle": 135.0},

    # Leg 4 (Left Middle)
    12: {"name": "Leg4_Link3", "physical_sid": 13, "direction":  1, "offset": -67.5,   "min_angle": -135.0, "max_angle": 135.0},
    13: {"name": "Leg4_Link2", "physical_sid": 14, "direction": -1, "offset": -43.875, "min_angle": -135.0, "max_angle": 135.0},
    14: {"name": "Leg4_Link1", "physical_sid": 15, "direction":  1, "offset":   0.0,   "min_angle": -135.0, "max_angle": 135.0},

    # Leg 5 (Left Front)
    15: {"name": "Leg5_Link3", "physical_sid": 16, "direction":  1, "offset": -67.5,   "min_angle": -135.0, "max_angle": 135.0},
    16: {"name": "Leg5_Link2", "physical_sid": 17, "direction": -1, "offset": -43.875, "min_angle": -135.0, "max_angle": 135.0},
    17: {"name": "Leg5_Link1", "physical_sid": 18, "direction":  1, "offset":   0.0,   "min_angle": -135.0, "max_angle": 135.0},
}