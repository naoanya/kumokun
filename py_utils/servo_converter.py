
class ServoConversionError(Exception):
    """Raised when conversion cannot be performed due to limits."""

class ServoConverter:
    """
    ICS Servo Value <-> Angle (Degree) Converter
    Range: 3500 (-135 deg) to 11500 (+135 deg), Neutral: 7500 (0 deg)
    """
    NEUTRAL_VALUE = 7500
    MIN_VALUE = 3500
    MAX_VALUE = 11500
    MAX_ANGLE_DEG = 135.0

    def __init__(
        self,
        direction: int = 1,
        offset: float = 0.0,
        min_angle: float | None = None,
        max_angle: float | None = None,
    ):
        """
        Initialize with direction and offset.
        :param direction: 1 for normal, -1 for reverse
        :param offset: Offset in degrees added to the target angle
        """
        self.direction = 1 if direction >= 0 else -1
        self.offset = offset
        # Optional software limits (degrees) for kinematic angle
        self.min_angle = float(min_angle) if min_angle is not None else None
        self.max_angle = float(max_angle) if max_angle is not None else None

    def convert_to_value(self, degrees: float) -> int:
        """Convert kinematic angle to servo value with direction and offset."""
        # Formula: servo_angle = (kinematic_angle * direction) + offset
        target_servo_angle = (degrees * self.direction) + self.offset
        # If software limits are set, enforce them on the servo-angle after direction/offset
        if (self.min_angle is not None) and (self.max_angle is not None):
            if not (self.min_angle <= target_servo_angle <= self.max_angle):
                raise ServoConversionError(
                    f"Servo angle {target_servo_angle} out of limits [{self.min_angle}, {self.max_angle}]"
                )
        return self.degrees_to_value(target_servo_angle)

    def convert_to_degrees(self, value: int) -> float:
        """Convert servo value to kinematic angle with direction and offset."""
        servo_angle = self.value_to_degrees(value)
        # Inverse Formula: kinematic_angle = (servo_angle - offset) * direction
        return (servo_angle - self.offset) * self.direction

    @staticmethod
    def degrees_to_value(degrees: float) -> int:
        """Convert degrees to servo value (raw conversion)."""
        # Calculate units per degree: (11500 - 7500) / 135 = 4000 / 135
        units = degrees * (4000.0 / 135.0)
        value = int(ServoConverter.NEUTRAL_VALUE + units)
        
        # Clamp to valid range
        return max(ServoConverter.MIN_VALUE, min(ServoConverter.MAX_VALUE, value))

    @staticmethod
    def value_to_degrees(value: int) -> float:
        """Convert servo value to degrees (raw conversion)."""
        # Calculate degrees per unit: 135 / 4000
        diff = value - ServoConverter.NEUTRAL_VALUE
        degrees = diff * (135.0 / 4000.0)
        return degrees