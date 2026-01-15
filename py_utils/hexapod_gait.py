"""Hexapod gait generator (CPG-based) and simple kinematic targets.

Produces per-leg foot targets (x, y, z, phase) in the robot body frame
for visualization or downstream trajectory execution. The controller
exposes simple runtime parameters (max speeds, stance duty, turn rate)
and a small state machine with `IDLE`, `WALKING`, and `HOMING` modes.

Timing: controller period is `DT` (default 0.01s).
"""

import math
from typing import List, Tuple, Final
from dataclasses import dataclass
from enum import Enum, auto

# =========================
# Constants
# =========================
DT = 0.01  # 10 ms control period.
TWO_PI = 2.0 * math.pi
# Threshold constants.
COMMAND_EPSILON = 1e-6  # Threshold for zero-command detection.
DISTANCE_EPSILON = 1e-8  # Threshold for zero-distance detection.
# Common leg count.
LEG_COUNT = 6
# Phase step for equally spaced leg offsets.
PHASE_STEP = TWO_PI / LEG_COUNT


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
    """Generate per-leg phase from the shared main phase.

    Supported `pattern` values:
        - 'tripod': two alternating tripods (legs 0/2/4 vs 1/3/5).
        - 'wave': evenly spaced 6-phase wave.
        - 'ripple': travelling ripple ordering.

    An explicit `phase_offsets` list (length 6) may be supplied to
    override the predefined patterns.
    """

    def __init__(self, pattern: str = "tripod", phase_offsets: List[float] = None):
        """Initialize leg phase generator.

        Args:
            pattern: one of 'tripod', 'wave', 'ripple' specifying common phase offsets.
            phase_offsets: optional explicit list of 6 phase offsets in radians (overrides pattern).
        """
        # predefined patterns (angles in radians)
        patterns = {
            "tripod": [0.0, math.pi, 0.0, math.pi, 0.0, math.pi],
            # evenly spaced 6-phase wave (0,60,120,180,240,300 deg)
            "wave": [i * PHASE_STEP for i in range(LEG_COUNT)],
            # ripple ordering: chosen index order to create a travelling ripple
            "ripple": [i * PHASE_STEP for i in [0, 1, 3, 2, 4, 5]],
        }

        if phase_offsets is not None:
            if len(phase_offsets) != LEG_COUNT:
                raise ValueError("phase_offsets must be a list of 6 values")
            self.phase_offsets: List[float] = phase_offsets
        else:
            self.phase_offsets: List[float] = patterns.get(pattern, patterns["tripod"])  # default tripod

        # rotation handling removed from leg-phase generator; handled by higher-level controller

    def get_phase(self, leg_index: int, main_phase: float) -> float:
        """
        Calculate leg phase from main phase.
        """
        # Left / Right leg discrimination is kept for patterns only (no rotation influence)
        phase = main_phase
        phase += self.phase_offsets[leg_index]
        phase %= TWO_PI
        return phase

# =========================
# FootTarget Data Class
# =========================
@dataclass
class FootTarget:
    """Foot target in the robot body frame.

    Attributes:
        x, y (float): planar coordinates in meters (body frame).
        z (float): vertical coordinate in meters (positive up).
        phase (float): CPG phase in radians (0 .. 2*pi).
    """
    x: float
    y: float
    z: float
    phase: float


@dataclass
class FootTargets:
    """Container for exactly six `FootTarget` objects.

    Provides simple validation at construction and a `copy()` helper for
    creating shallow snapshots of the current targets.
    """
    targets: List[FootTarget]

    def __post_init__(self):
        if len(self.targets) != 6:
            raise ValueError("FootTargets.targets must contain 6 FootTarget elements")

    def copy(self) -> "FootTargets":
        """Return a shallow copy of this FootTargets container."""
        return FootTargets([FootTarget(x=ft.x, y=ft.y, z=ft.z, phase=ft.phase) for ft in self.targets])


# =========================
# State Enum
# =========================

# State machine for high-level gait modes
class State(Enum):
    IDLE = auto()
    WALKING = auto()
    HOMING = auto()


# =========================
# Hexapod Gait Controller
# =========================
class HexapodGait:
    """
    Hexapod gait controller using separated CPG phases.

    Produces per-leg `FootTarget` outputs and maintains a simple state
    machine that supports `IDLE`, `WALKING`, and `HOMING` behaviours.
    """

    def __init__(self, gait: str = "tripod"):
        """HexapodGait constructor.

        Args:
            gait: initial gait pattern name ('tripod', 'wave', 'ripple')
        """
        self.main_phase_gen = MainPhaseGenerator()
        # initialize leg phase generator with requested gait pattern
        self.leg_phase_gen = LegPhaseGenerator(pattern=gait)

        # Angles for LF, LM, LR, RF, RM, RR
        leg_home_radius = 0.18  # [m] leg home radius (body-relative)
        mount_angles = [-30, -90, -150, 30, 90, 150]
        home_list = [
            FootTarget(x=(leg_home_radius * math.cos(math.radians(a))),
                       y=(leg_home_radius * math.sin(math.radians(a))),
                       z=0.0,
                       phase=0.0)
            for a in mount_angles
        ]
        self.leg_home_positions: Final[FootTargets] = FootTargets(home_list)
        # `active_targets` holds the runtime foot targets (initialized to home).
        self.active_targets = self.leg_home_positions.copy()

        # Trajectory parameters
        self.step_height = 0.03  # [m]
        # maximum forward speed mapping: speed (0..1) -> body velocity [m/s]
        self.max_forward_speed = 0.2  # [m/s]
        # separate swing speed (m/s) and per-step coefficient for swing leg movement
        self.max_swing_speed = self.max_forward_speed  # default: same as forward speed
        # Store last computed phases.
        self.global_phase = 0.0
        self.per_leg_phases = [0.0 for _ in range(LEG_COUNT)]
        # rotation parameters (instance-level, adjustable at runtime)
        self.max_turn_rate = 1.0  # [rad/s] when rotation == 1.0
        # input smoothing (to tolerate sudden direction/rotation changes)
        # time constants (seconds)
        self.direction_tau = 0.08
        self.rotation_tau = 0.06
        self.speed_tau = 0.08
        # filtered inputs
        self.filtered_direction = 0.0
        self.filtered_rotation = 0.0
        self.filtered_speed = 0.0
        # stance fraction: proportion of cycle spent in stance (0..1). 0.5 == equal swing/stance
        self.stance_fraction = 0.5
        # homing timeout: if speed and rotation stay zero for this many seconds, go to HOMING
        self.homing_timeout = 0.5  # seconds
        self.zero_cmd_accum = 0.0
        # fixed drive speed used during HOMING (normalized 0..1)
        self.homing_drive_speed = 0.2
        # Per-leg flag: True if leg has reached home and is grounded (used in HOMING).
        self.leg_homed_and_grounded = [False for _ in range(LEG_COUNT)]
        # Threshold distance to consider leg "at home" (meters).
        self.homing_threshold = 0.005
        # raw input values (for state transition detection)
        self.raw_speed = 0.0
        self.raw_rotation = 0.0
        # Controller state.
        self.state: State = State.IDLE

        # compute derived per-step coefficients from tunable maxima
        self._recompute_coeffs()

    # -------------------------
    # Coefficient and parameter helpers
    # -------------------------
    def _recompute_coeffs(self) -> None:
        """Recompute derived, per-control-step coefficients.

        Converts user-facing maxima (m/s, rad/s) into per-step increments
        used by the controller. Call whenever a max parameter is updated.
        """
        self.step_move_coeff = self.max_forward_speed * DT
        self.swing_step_coeff = self.max_swing_speed * DT
        self.rotation_coeff = self.max_turn_rate * DT

    def set_max_forward_speed(self, val: float) -> None:
        """Set `max_forward_speed` (m/s) and recompute dependent coeffs."""
        if val < 0.0:
            raise ValueError("max_forward_speed must be non-negative")
        self.max_forward_speed = float(val)
        self._recompute_coeffs()

    def set_max_swing_speed(self, val: float) -> None:
        """Set `max_swing_speed` (m/s) and recompute dependent coeffs."""
        if val < 0.0:
            raise ValueError("max_swing_speed must be non-negative")
        self.max_swing_speed = float(val)
        self._recompute_coeffs()

    def set_max_turn_rate(self, val: float) -> None:
        """Set `max_turn_rate` (rad/s) and recompute dependent coeffs."""
        if val < 0.0:
            raise ValueError("max_turn_rate must be non-negative")
        self.max_turn_rate = float(val)
        self._recompute_coeffs()

    def set_stance_fraction(self, val: float) -> None:
        """Set `stance_fraction` (0..1). Clamped to (0.01..0.99)."""
        v = float(val)
        if v <= 0.0 or v >= 1.0:
            raise ValueError("stance_fraction must be between 0 and 1 (exclusive)")
        self.stance_fraction = v

    # -------------------------
    # Phase -> geometry helpers
    # -------------------------
    def _leg_height_from_phase(self, phase: float) -> float:
        """Return z height for a leg given its phase (uses smoothstep)."""
        if self._phase_to_grounded(phase):
            return 0.0
        prog = self._phase_to_progress(phase)
        smooth_p = self._smooth_step(prog)
        return self.step_height * smooth_p

    # -------------------------
    # Naming-compatible wrappers
    # -------------------------
    def step(self, speed: float, direction: float, rotation: float) -> "FootTargets":
        """Perform one control step and return current foot targets.

        This is the main entry point intended to be called by the
        upper layer at period `DT` (default 10 ms). It updates raw and
        filtered inputs, then dispatches to the handler for the current
        state (`IDLE`, `WALKING`, `HOMING`).
        """

        # Store raw input values for state-transition detection.
        self.raw_speed = speed
        self.raw_rotation = rotation

        # Update filtered inputs.
        self._filter_direction(direction)
        self._filter_rotation(rotation)
        self._filter_speed(speed)

        # Dispatch to per-state handlers.
        if self.state == State.IDLE:
            return self._loop_idle(self.filtered_speed, self.filtered_direction, self.filtered_rotation)
        elif self.state == State.WALKING:
            return self._loop_walking(self.filtered_speed, self.filtered_direction, self.filtered_rotation)
        elif self.state == State.HOMING:
            return self._loop_homing(self.filtered_speed, self.filtered_direction, self.filtered_rotation)
        else:
            return self._loop_idle(self.filtered_speed, self.filtered_direction, self.filtered_rotation)

    # -------------------------
    # Private helper methods
    # -------------------------

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

    def _filter_speed(self, speed: float) -> None:
        """First-order low-pass filter for speed command.

        Updates `self.filtered_speed` in-place.
        """
        if self.speed_tau <= 0.0:
            self.filtered_speed = speed
            return

        alpha_spd = DT / (self.speed_tau + DT)
        self.filtered_speed += alpha_spd * (speed - self.filtered_speed)

    def _smooth_step(self, p: float) -> float:
        """5th-order smoothstep: maps p in [0,1] -> smoothed p with zero endpoint derivatives.

        Formula: 6p^5 - 15p^4 + 10p^3 = p^3*(p*(6p - 15) + 10)
        """
        p = max(0.0, min(1.0, p))
        return p * p * p * (p * (p * 6.0 - 15.0) + 10.0)

    def _phase_to_progress(self, phase: float) -> float:
        """Return normalized progress (0..1) within current swing or stance window.

        With configurable `stance_fraction` the stance window length is
        `2*pi*stance_fraction` and the swing window is the remainder. This
        function returns progress within whichever window the phase falls
        into (0=start of that window, 1=end).
        """
        p = phase % TWO_PI
        # threshold where stance begins (end of swing)
        stance_start = TWO_PI * (1.0 - self.stance_fraction)
        if p < stance_start:
            # in swing window
            if stance_start <= 0.0:
                return 0.0
            return p / stance_start
        else:
            # in stance window
            stance_len = TWO_PI - stance_start
            if stance_len <= 0.0:
                return 0.0
            return (p - stance_start) / stance_len

    def _phase_to_grounded(self, phase: float) -> bool:
        """Return True if phase is in stance (grounded), False if in swing.

        Stance start is computed from `stance_fraction`. For the default
        `stance_fraction == 0.5` this matches the previous behaviour
        (stance when phase >= pi).
        """
        p = phase % TWO_PI
        stance_start = TWO_PI * (1.0 - self.stance_fraction)
        return p >= stance_start

    # -------------------------
    # Common gait update methods
    # -------------------------

    def _update_phases(self, drive_speed: float) -> None:
        """Update main phase and per-leg phases based on drive speed."""
        self.global_phase = self.main_phase_gen.update(drive_speed)
        for i in range(LEG_COUNT):
            self.per_leg_phases[i] = self.leg_phase_gen.get_phase(i, self.global_phase)
        # Reflect computed phases into runtime foot targets.
        for i in range(LEG_COUNT):
            self.active_targets.targets[i].phase = self.per_leg_phases[i]

    def _compute_movement_params(self, speed: float, direction: float, rotation: float) -> Tuple[float, float, float, float]:
        """Compute movement parameters for stance legs.

        Returns:
            (neg_dx, neg_dy, rot_cos, rot_sin)
        """
        # All linear displacements are in meters per control step. Rotation
        # terms are cos/sin of the per-step rotation angle (radians).
        move_mag = speed * self.step_move_coeff
        dir_cos = math.cos(direction)
        dir_sin = math.sin(direction)
        neg_dx = -move_mag * dir_cos
        neg_dy = -move_mag * dir_sin
        rot_ang = rotation * self.rotation_coeff
        rot_cos = math.cos(rot_ang)
        rot_sin = math.sin(rot_ang)
        return neg_dx, neg_dy, rot_cos, rot_sin

    def _update_grounded_leg(self, leg_index: int, neg_dx: float, neg_dy: float, rot_cos: float, rot_sin: float) -> None:
        """Update a grounded (stance) leg position: translate and rotate."""
        cp = self.active_targets.targets[leg_index]
        # Apply translation into temporaries to avoid reloading mutated fields.
        tx = cp.x + neg_dx
        ty = cp.y + neg_dy
        # Rotate around origin using precomputed cos/sin.
        x_new = tx * rot_cos - ty * rot_sin
        y_new = tx * rot_sin + ty * rot_cos
        cp.x, cp.y = x_new, y_new

    def _update_swing_leg(self, leg_index: int, step_mag: float) -> None:
        """Update a swing leg position: move toward home position."""
        cp = self.active_targets.targets[leg_index]
        hp = self.leg_home_positions.targets[leg_index]
        # Compute vector to home and distance.
        dx_home = hp.x - cp.x
        dy_home = hp.y - cp.y
        dist = math.hypot(dx_home, dy_home)
        if dist <= DISTANCE_EPSILON:
            # Snap to home if effectively at the target.
            cp.x, cp.y = hp.x, hp.y
        else:
            # Move up to `step_mag` toward home this control step.
            step = min(dist, step_mag)
            ratio = step / dist
            cp.x += dx_home * ratio
            cp.y += dy_home * ratio

    def _update_leg_heights(self) -> None:
        """Update z coordinate for all legs based on phase (swing raised, stance ground)."""
        for i in range(LEG_COUNT):
            cp = self.active_targets.targets[i]
            phase = self.per_leg_phases[i]
            # Set z from phase using the shared height helper.
            cp.z = self._leg_height_from_phase(phase)

    def _is_at_home(self, leg_index: int) -> bool:
        """Return True when the leg is within `homing_threshold` of its home.

        This compares the planar (x,y) distance to the configured threshold.
        """
        cp = self.active_targets.targets[leg_index]
        hp = self.leg_home_positions.targets[leg_index]
        dist = math.hypot(cp.x - hp.x, cp.y - hp.y)
        return dist <= self.homing_threshold

    def _create_foot_targets_copy(self) -> "FootTargets":
        """Return a shallow copy of the current active targets."""
        return FootTargets([
            FootTarget(x=ft.x, y=ft.y, z=ft.z, phase=ft.phase)
            for ft in self.active_targets.targets
        ])

    # -------------------------
    # State handlers
    # -------------------------
    def _loop_idle(self, speed: float, direction: float, rotation: float) -> "FootTargets":
        """Idle state: stand at home positions, transition to WALKING on movement command."""
        # If a movement command appears, switch to WALKING immediately.
        if abs(speed) > COMMAND_EPSILON or abs(rotation) > COMMAND_EPSILON:
            # Reinitialize phase generators so walking starts from a clean CPG state.
            self.main_phase_gen = MainPhaseGenerator()
            # Recreate leg_phase_gen preserving configured offsets.
            self.leg_phase_gen = LegPhaseGenerator(phase_offsets=list(self.leg_phase_gen.phase_offsets))
            # Reset stored phases.
            self.global_phase = 0.0
            self.per_leg_phases = [0.0 for _ in range(LEG_COUNT)]

            # Initialize active targets to home positions when starting to walk.
            self.active_targets = self.leg_home_positions.copy()
            # Reset leg_homed_and_grounded flags.
            self.leg_homed_and_grounded = [False for _ in range(LEG_COUNT)]
            self.state = State.WALKING
            return self._loop_walking(speed, direction, rotation)

        # Return the stored home positions directly (simple, non-copying).
        return self.leg_home_positions

    def _loop_walking(self, speed: float, direction: float, rotation: float) -> "FootTargets":
        """Walking state: generate walking gait using CPG phases."""
        # Drive main phase based on input speed or rotation magnitude (whichever is larger).
        # If raw speed and rotation remain zero for a while, transition to HOMING.
        # Use raw values (not filtered) for state-transition detection.
        if abs(self.raw_speed) <= COMMAND_EPSILON and abs(self.raw_rotation) <= COMMAND_EPSILON:
            self.zero_cmd_accum += DT
        else:
            self.zero_cmd_accum = 0.0

        if self.zero_cmd_accum >= self.homing_timeout:
            self.zero_cmd_accum = 0.0
            # Reset leg_homed_and_grounded flags before entering HOMING.
            self.leg_homed_and_grounded = [False for _ in range(LEG_COUNT)]
            self.state = State.HOMING
            return self._loop_homing(speed, direction, rotation)

        drive_speed = max(speed, abs(rotation))
        self._update_phases(drive_speed)

        # Compute movement parameters
        neg_dx, neg_dy, rot_cos, rot_sin = self._compute_movement_params(speed, direction, rotation)
        step_mag = drive_speed * self.swing_step_coeff

        # Update runtime foot positions
        for i in range(LEG_COUNT):
            grounded = self._phase_to_grounded(self.per_leg_phases[i])
            if grounded:
                self._update_grounded_leg(i, neg_dx, neg_dy, rot_cos, rot_sin)
            else:
                self._update_swing_leg(i, step_mag)

        self._update_leg_heights()

        return self._create_foot_targets_copy()

    def _loop_homing(self, speed: float, direction: float, rotation: float) -> "FootTargets":
        """Homing state: move all legs to home positions and transition to IDLE when complete.

        Behavior:
        - Legs that have reached home position and are grounded stay grounded regardless of phase.
        - When all legs are at home and grounded, transition to IDLE state.
        """
        drive_speed = self.homing_drive_speed
        self._update_phases(drive_speed)

        step_mag = drive_speed * self.swing_step_coeff

        # Update runtime foot positions with homing logic.
        for i in range(LEG_COUNT):
            cp = self.active_targets.targets[i]
            phase_grounded = self._phase_to_grounded(self.per_leg_phases[i])

            # Check whether the leg has reached home and is currently grounded.
            if self._is_at_home(i) and phase_grounded:
                self.leg_homed_and_grounded[i] = True

            if self.leg_homed_and_grounded[i]:
                # Once homed and grounded, stay at home position on ground.
                hp = self.leg_home_positions.targets[i]
                cp.x, cp.y, cp.z = hp.x, hp.y, 0.0
            else:
                # Normal gait logic.
                if phase_grounded:
                    # Grounded but not yet at home: hold current position.
                    pass
                else:
                    # Swing leg: move toward home.
                    self._update_swing_leg(i, step_mag)

        # Update z for non-homed legs.
        for i in range(LEG_COUNT):
            if not self.leg_homed_and_grounded[i]:
                cp = self.active_targets.targets[i]
                phase = self.per_leg_phases[i]
                cp.z = self._leg_height_from_phase(phase)

        # Check if all legs are homed and grounded.
        if all(self.leg_homed_and_grounded):
            self.state = State.IDLE
            # Reset homed flags for next homing cycle.
            self.leg_homed_and_grounded = [False for _ in range(LEG_COUNT)]
            return self.leg_home_positions

        return self._create_foot_targets_copy()
