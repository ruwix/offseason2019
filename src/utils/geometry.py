import numpy as np


class RobotState:
    def __init__(self, x: float, y: float, heading: float, v: float, omega: float):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = v
        self.omega = omega

    def update(self, last_state: RobotState, dt: float) -> None:
        dx = self.x - last_state.x
        dy = self.y - last_state.y
        dheading = self.heading - last_state.heading
        self.v = np.hypot(dx, dy) / dt
        self.omega = dheading / dt
