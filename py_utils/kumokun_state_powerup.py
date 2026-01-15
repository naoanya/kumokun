import time


class PowerupManager:
    """Simple manager for POWERUP state.

    Performs any initialization tasks required before transitioning to IDLE.
    This implementation is intentionally minimal: it provides a start/update
    interface and a short internal delay to simulate initialization work.
    """

    def __init__(self, controller, kinematics=None, update_interval: float = 0.05, duration: float = 0.5):
        self.controller = controller
        self.kinematics = kinematics
        self.update_interval = update_interval
        self.duration = duration
        self.state = "IDLE"
        self.start_time = 0.0

    def start(self):
        self.state = "ENTER"
        self.start_time = time.time()

    def update(self):
        if self.state == "ENTER":
            if time.time() - self.start_time >= self.duration:
                self.state = "DONE"
        return True
