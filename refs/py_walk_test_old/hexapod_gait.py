import math
from typing import Dict, List, Tuple

# =========================
# Constants
# =========================
DT = 0.01  # 10 ms control period
TWO_PI = 2.0 * math.pi
# Rotation / turning parameters
MAX_TURN_RATE = 1.0  # [rad/s] when rotation == 1.0
TURN_SCALE = 0.02    # dimensionless scale applied to (omega * radius * DT) to compute displacement [m]


# =========================
# Main Phase Generator
# =========================
class MainPhaseGenerator:
    """
    Global CPG phase generator.
    Generates a single walking phase for the whole robot.
    """

    def __init__(self, base_frequency: float = 2.0):
        self.phase = 0.0
        self.base_frequency = base_frequency  # [Hz] at speed = 2.0

    def update(self, speed: float) -> float:
        """
        Update and return main phase (0 ~ 2*pi).
        """
        speed = max(0.0, min(1.0, speed))

        omega = TWO_PI * self.base_frequency * speed
        self.phase += omega * DT
        self.phase %= TWO_PI

        return self.phase


# =========================
# Leg Phase Generator
# =========================
class LegPhaseGenerator:
    """
    Generates per-leg phase from main phase.
    """

    LEG_COUNT = 6

    def __init__(self, pattern: str = "tripod", phase_offsets: List[float] = None):
        """Initialize leg phase generator.

        Args:
            pattern: one of 'tripod', 'wave', 'ripple' specifying common phase offsets.
            phase_offsets: optional explicit list of 6 phase offsets in radians (overrides pattern).
        """
        # predefined patterns (angles in radians)
        patterns = {
            "tripod": [0.0, math.pi, 0.0, math.pi, 0.0, math.pi],
            "wave": [0.0, TWO_PI / 3.0, 2.0 * TWO_PI / 3.0, math.pi, 4.0 * TWO_PI / 3.0, 5.0 * TWO_PI / 3.0],
            "ripple": [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0, math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0],
        }

        if phase_offsets is not None:
            if len(phase_offsets) != self.LEG_COUNT:
                raise ValueError("phase_offsets must be a list of 6 values")
            self.phase_offsets: List[float] = phase_offsets
        else:
            self.phase_offsets: List[float] = patterns.get(pattern, patterns["tripod"])  # default tripod

        # Rotation phase gain (tunable)
        self.rotation_gain = 0.3  # [rad]

    def get_phase(
        self,
        leg_index: int,
        main_phase: float,
        rotation: float,
    ) -> float:
        """
        Calculate leg phase from main phase.
        """
        rotation = max(-1.0, min(1.0, rotation))

        # Left / Right leg discrimination
        side = -1.0 if leg_index < 3 else 1.0

        phase = main_phase
        phase += self.phase_offsets[leg_index]
        phase += rotation * side * self.rotation_gain
        phase %= TWO_PI

        return phase


# =========================
# Hexapod Gait Controller
# =========================
class HexapodGait:
    """
    Hexapod gait controller using separated CPG phases.
    """

    LEG_COUNT = 6

    def __init__(self, gait: str = "tripod"):
        """HexapodGait constructor.

        Args:
            gait: initial gait pattern name ('tripod', 'wave', 'ripple')
        """
        self.main_phase = MainPhaseGenerator()
        # initialize leg phase generator with requested gait pattern
        self.leg_phase = LegPhaseGenerator(pattern=gait)

        # leg mount positions (body-relative) for turning computations
        self.leg_mount_radius = 0.18
        # Angles for LF, LM, LR, RF, RM, RR
        mount_angles = [-60, -120, 180, 60, 120, 0]
        self.leg_mounts = [
            (self.leg_mount_radius * math.cos(math.radians(a)), self.leg_mount_radius * math.sin(math.radians(a)))
            for a in mount_angles
        ]

        # Trajectory parameters
        self.step_length = 0.15   # [m]
        self.step_height = 0.03  # [m]
        # maximum forward speed mapping: speed (0..1) -> body velocity [m/s]
        self.max_forward_speed = 0.2  # [m/s]
        # store last computed phases
        self.current_main_phase = 0.0
        self.leg_phases = [0.0 for _ in range(self.LEG_COUNT)]
        # per-leg recorded heading at touchdown (rad)
        self.stance_heading = [0.0 for _ in range(self.LEG_COUNT)]
        # previous grounded state for edge detection
        self.prev_grounded = [False for _ in range(self.LEG_COUNT)]
        # per-leg recorded horizontal displacement for stance (m)
        self.stance_displacement = [(0.0, 0.0) for _ in range(self.LEG_COUNT)]
        # rotation parameters (instance-level, adjustable at runtime)
        self.max_turn_rate = MAX_TURN_RATE
        self.turn_scale = TURN_SCALE
        # input smoothing (to tolerate sudden direction/rotation changes)
        # time constants (seconds)
        self.direction_tau = 0.08
        self.rotation_tau = 0.06
        # filtered inputs
        self.filtered_direction = 0.0
        self.filtered_rotation = 0.0

    def loop(
        self,
        speed: float,
        direction: float,
        rotation: float,
    ) -> Dict[int, Dict[str, float]]:
        """
        Main control loop.
        Called every 10 ms by upper layer.

        Args:
            speed:     0.0 ~ 1.0
            direction: -pi ~ +pi (rad)
            rotation:  -1.0 ~ +1.0

        Returns:
            Dict[leg_index] -> foot target (x, y, z, phase)
        """

        main_phase = self.main_phase.update(speed)
        # store main phase
        self.current_main_phase = main_phase

        # apply first-order low-pass to direction and rotation to avoid abrupt changes
        self._filter_direction(direction)
        self._filter_rotation(rotation)

        # Initialize filtered inputs on first run to avoid startup jump
        if not hasattr(self, "_inited") or not self._inited:
            self.filtered_direction = direction
            self.filtered_rotation = rotation
            self._inited = True

        # heading for swing uses current filtered_direction; stance heading is captured at touchdown
        dx_swing = math.cos(self.filtered_direction)
        dy_swing = math.sin(self.filtered_direction)

        # compute cycle period (s); if speed==0 use base frequency period
        if speed > 1e-6:
            cycle_period = 1.0 / (self.main_phase.base_frequency * speed)
        else:
            cycle_period = 1.0 / max(1e-6, self.main_phase.base_frequency)

        # global per-step displacement candidate (m) to be used for swing endpoints
        per_step_disp_global = self.step_length * max(0.0, min(1.0, speed))

        outputs: Dict[int, Dict[str, float]] = {}

        # compute stance duty ratio based on speed (higher speed -> shorter stance)
        stance_duty = self._get_stance_duty(speed)

        for leg in range(self.LEG_COUNT):
            phase = self.leg_phase.get_phase(leg, main_phase, self.filtered_rotation)
            # store per-leg phase
            self.leg_phases[leg] = phase

            # compute normalized phase progress and grounded flag using duty ratio
            phase_progress = self._phase_to_progress_with_duty(phase, stance_duty)
            is_grounded = self._phase_to_grounded_with_duty(phase, stance_duty)

            if is_grounded:
                # on touchdown, capture heading and horizontal displacement for this stance
                if not self.prev_grounded[leg]:
                    self.stance_heading[leg] = self.filtered_direction
                    hx = math.cos(self.stance_heading[leg])
                    hy = math.sin(self.stance_heading[leg])
                    # per-step displacement: proportional to requested speed (clamped)
                    per_step_disp = per_step_disp_global
                    self.stance_displacement[leg] = (hx * per_step_disp, hy * per_step_disp)

                disp_x, disp_y = self.stance_displacement[leg]
                x_base, y_base, z = self._compute_stance_target(phase_progress, disp_x, disp_y)
            else:
                x_base, y_base, z = self._compute_swing_target(phase_progress, dx_swing, dy_swing)

            # apply rotation-induced tangential offset only when foot is grounded
            if is_grounded:
                rot_dx, rot_dy = self._rotation_offset_for_leg(leg, cycle_period)
            else:
                rot_dx, rot_dy = 0.0, 0.0

            x = x_base + rot_dx
            y = y_base + rot_dy

            outputs[leg] = {"x": x, "y": y, "z": z, "phase": phase}

            # update previous grounded state
            self.prev_grounded[leg] = is_grounded

        return outputs

    def _filter_direction(self, direction: float) -> None:
        """First-order low-pass filter for angular direction with wrap-around handling.

        Updates `self.filtered_direction` in-place.
        """
        if self.direction_tau <= 0.0:
            self.filtered_direction = direction
            return

        alpha_dir = DT / (self.direction_tau + DT)
        # handle angular wrap-around for shortest-path smoothing
        delta_dir = (direction - self.filtered_direction + math.pi) % (2 * math.pi) - math.pi
        self.filtered_direction += alpha_dir * delta_dir

    def _filter_rotation(self, rotation: float) -> None:
        """First-order low-pass filter for rotation command.

        Updates `self.filtered_rotation` in-place.
        """
        if self.rotation_tau <= 0.0:
            self.filtered_rotation = rotation
            return

        alpha_rot = DT / (self.rotation_tau + DT)
        self.filtered_rotation += alpha_rot * (rotation - self.filtered_rotation)

    def _compute_swing_target(self, phase_progress: float, disp_x: float, disp_y: float) -> Tuple[float, float, float]:
        """Compute swing foot target (x,y,z) from normalized progress and planned horizontal displacement.

        Args:
            disp_x, disp_y: horizontal displacement over one step (m) in heading-aligned axes.

        Returns tuple (x[m], y[m], z[m]).
        """
        smooth_p = self._smooth_step(phase_progress)
        x = disp_x * (smooth_p - 0.5)
        y = disp_y * (smooth_p - 0.5)
        z = self.step_height * smooth_p
        return x, y, z

    def _smooth_step(self, p: float) -> float:
        """5th-order smoothstep: maps p in [0,1] -> smoothed p with zero endpoint derivatives.

        Formula: 6p^5 - 15p^4 + 10p^3 = p^3*(p*(6p - 15) + 10)
        """
        p = max(0.0, min(1.0, p))
        return p * p * p * (p * (p * 6.0 - 15.0) + 10.0)

    def _compute_stance_target(self, phase_progress: float, dx: float, dy: float) -> Tuple[float, float, float]:
        """Compute stance foot target (x,y,z) from normalized progress and horizontal displacement.

        Args:
            dx, dy: horizontal displacement over one step (m) captured at touchdown.

        Returns tuple (x[m], y[m], z[m]).
        """
        # dx,dy are already a displacement over one step (m)
        x = dx * (0.5 - phase_progress)
        y = dy * (0.5 - phase_progress)
        z = 0.0
        return x, y, z

    def _rotation_offset_for_leg(self, leg: int, cycle_period: float) -> Tuple[float, float]:
        """Compute rotation-induced tangential offset for given leg (x,y).

        Calculation details / units:
          - filtered_rotation: normalized input in [-1,1]
          - max_turn_rate: [rad/s] when normalized rotation == 1.0
          - omega = filtered_rotation * max_turn_rate -> [rad/s]
          - omega * radius -> tangential speed [m/s]
          - multiply by DT -> displacement over one control step [m]
          - multiply by turn_scale (dimensionless tuning factor)

        Returns (dx[m], dy[m]).
        """
        omega = self.filtered_rotation * self.max_turn_rate  # [rad/s]
        bx, by = self.leg_mounts[leg]  # [m]
        # tangential displacement over one cycle (m) scaled by turn_scale
        rot_dx = -omega * by * cycle_period * self.turn_scale
        rot_dy = omega * bx * cycle_period * self.turn_scale
        return rot_dx, rot_dy

    def _get_stance_duty(self, speed: float) -> float:
        """Return stance duty ratio (fraction of cycle spent in stance) based on speed.

        Mapping: at speed=0 -> long stance (0.75), at speed=1 -> shorter stance (0.5).
        Clamped to [0.1, 0.9] for safety.
        """
        duty = 0.75 - 0.25 * max(0.0, min(1.0, speed))
        return max(0.1, min(0.9, duty))

    def _phase_to_progress_with_duty(self, phase: float, stance_duty: float) -> float:
        """Return normalized progress (0..1) within current swing or stance, given stance duty.

        Assumes phase in [0, 2π). Defines stance to occupy the final `stance_duty` fraction
        of the cycle (i.e. phase_norm in [1-stance_duty, 1)).
        """
        phase_norm = (phase % TWO_PI) / TWO_PI  # 0..1
        swing_fraction = max(1e-6, 1.0 - stance_duty)
        if phase_norm < (1.0 - stance_duty):
            # swing: progress 0..1 across swing_fraction
            return phase_norm / swing_fraction
        else:
            # stance: progress 0..1 across stance_duty
            return (phase_norm - (1.0 - stance_duty)) / stance_duty

    def _phase_to_grounded_with_duty(self, phase: float, stance_duty: float) -> bool:
        """Return True if phase is in stance (grounded) according to stance_duty."""
        phase_norm = (phase % TWO_PI) / TWO_PI
        return phase_norm >= (1.0 - stance_duty)

    def _phase_to_progress(self, phase: float) -> float:
        """Return normalized progress (0..1) within current half-cycle (swing or stance)."""
        if phase < math.pi:
            return phase / math.pi
        return (phase - math.pi) / math.pi

    def _phase_to_grounded(self, phase: float) -> bool:
        """Return True if phase is in stance (grounded), False if in swing."""
        return phase >= math.pi
