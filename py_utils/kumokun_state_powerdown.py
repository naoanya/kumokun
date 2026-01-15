import time


class PowerdownManager:
    """Manager for POWERDOWN state.

    Performs graceful shutdown actions such as freeing servos.
    This implementation is intentionally conservative and will only send
    an AF (free all) command once on start.
    """

    def __init__(self, controller, kinematics=None, update_interval: float = 0.05):
        self.controller = controller
        self.kinematics = kinematics
        self.update_interval = update_interval
        self.state = "IDLE"
        self.started = False

    def start(self):
        self.state = "ENTER"
        self.started = False

    def update(self):
        if self.state == "ENTER" and not self.started:
            try:
                self.controller.free_all()
            except Exception:
                pass
            self.started = True
            self.state = "DONE"
        return True
