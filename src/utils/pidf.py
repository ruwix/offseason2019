class PIDF:
    """A PIDF skeleton class."""

    EPSILON = 1e-6

    def __init__(self, kp=0, ki=0, kd=0, kf=0, continuous=False, minIn=-100, maxIn=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf

        self.last_error = 0
        self.cur_error = 0

        self.integral = 0
        self.derivative = 0

        self.output = 0
        self.continuous = continuous

        self.minIn = minIn
        self.maxIn = maxIn

    def update(self, setpoint, input, dt):
        """Update the PIDF controller."""
        self.setpoint = setpoint
        if dt < self.EPSILON:
            dt = self.EPSILON
        self.cur_error = self.setpoint - input
        if self.continuous and abs(self.cur_error) > (self.maxIn - self.minIn) / 2:
            if self.cur_error > 0:
                self.cur_error = self.cur_error - self.maxIn + self.minIn
            else:
                self.cur_error = self.cur_error + self.maxIn - self.minIn
        self.proportion = self.cur_error
        self.integral = self.integral + (self.cur_error * dt)
        self.derivative = (self.cur_error - self.last_error) / dt
        self.output = (
            (self.kp * self.proportion)
            + (self.ki * self.integral)
            + (self.kd * self.derivative)
            + (self.kf * self.setpoint)
        )
        return self.output

    def reset(self):
        """Reset control values."""
        self.last_error = 0
        self.cur_error = 0

        self.integral = 0
        self.derivative = 0

        self.setpoint = 0
        self.output = 0
