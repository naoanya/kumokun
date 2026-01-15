from enum import Enum, auto


class RobotState(Enum):
    POWERUP = auto()
    IDLE = auto()
    HOMING = auto()
    STANDUP = auto()
    WALK_MODE0 = auto()
    WALK_MODE1 = auto()
    STANDDOWN = auto()
    POWERDOWN = auto()
