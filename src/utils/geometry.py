import numpy as np


class RobotState:
    def __init__(self, x, y, heading, v, omega):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = v
        self.omega = omega

    def update(self, last_state, dt):
        dx = self.x - last_state.x
        dy = self.y - last_state.y
        dheading = self.heading - last_state.heading
        self.v = np.hypot(dx, dy) / dt
        self.omega = dheading / dt
        
