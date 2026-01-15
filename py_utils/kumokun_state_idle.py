class IdleManager:
    """Minimal manager for IDLE state.

    Remains idle until external commands change state. Provides a consistent
    `start()` / `update()` API to be used by the controller.
    """

    def __init__(self, controller, kinematics=None, update_interval: float = 0.05):
        self.controller = controller
        self.kinematics = kinematics
        self.update_interval = update_interval
        self.state = "IDLE"

    def start(self):
        self.state = "ENTER"

    def update(self):
        # Nothing to do for idle; keep returning True
        return True
