from utils.epsilon import epsilonEquals


class RealDifferentialDrive:
    EPSILON = 1e-3

    def __init__(self, gearbox, gear_ratio, track_width, wheel_radius, mass, moi):
        self.gearbox = gearbox
        self.gear_ratio = gear_ratio
        self.track_width = track_width
        self.wheel_radius = wheel_radius
        self.mass = mass
        self.moi = moi
        self.al = 0
        self.ar = 0
        self.vl = 0
        self.vr = 0
        self.fl = 0
        self.fr = 0
        self.c1 = -(gear_ratio ** 2 * self.gearbox.kt) / (
            self.gearbox.kv * self.gearbox.resistance * self.wheel_radius ** 2
        )
        self.c2 = (self.gear_ratio * self.gearbox.kt) / (
            self.gearbox.resistance * self.wheel_radius
        )

    def update(self, voltage_l, voltage_r, dt):
        self.fl = self.c1 * self.vl + self.c2 * voltage_l
        self.fr = self.c1 * self.vr + self.c2 * voltage_r
        positive_term = 1 / self.mass + self.track_width ** 2 / self.moi
        negative_term = 1 / self.mass - self.track_width ** 2 / self.moi

        self.al = positive_term * self.fl + negative_term * self.fr
        self.ar = negative_term * self.fl + positive_term * self.fr
        self.al = 0 if epsilonEquals(self.al, 0, self.EPSILON) else self.al
        self.ar = 0 if epsilonEquals(self.ar, 0, self.EPSILON) else self.ar
        self.vl += self.al * dt
        self.vr += self.ar * dt
