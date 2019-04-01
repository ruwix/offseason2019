import numpy as np


def boundHalfRadians(theta):
    while theta >= np.pi:
        theta -= 2 * np.pi
    while theta < -np.pi:
        theta += 2 * np.pi
    return theta


class Vector:
    """Simple 2d vector."""

    def __init__(self, x: float = 0, y: float = 0):
        self.x = x
        self.y = y

    def getDistance(self, other) -> float:
        """Compute the distance between 2 vectors."""
        return (self - other).getMagnitude()

    def getMagnitude(self) -> float:
        """Get the magnitude (distance to origin) of the vector."""
        return np.hypot(self.x, self.y)

    def getArgument(self) -> float:
        """Get the argument (angle from the postive axis) of the vector."""
        return np.arctan2(self.x, self.y)

    def getNormalized(self) -> float:
        """Get the normalized (unit) vector (a vector with the same argument but a magntitude of 1)."""
        return self / self.getMagnitude()

    def getRotated(self, theta: float):
        """Get a vector that has been rotated about the origin by theta."""
        st, ct = np.sin(theta), np.cos(theta)
        x = (self.x * ct) + (self.y * st)
        y = -(self.x * st) + (self.y * ct)
        return Vector(x, y)

    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and (self.x == other.x)
            and (self.y == other.y)
        )

    def __add__(self, other):
        if isinstance(other, self.__class__):
            x = self.x + other.x
            y = self.y + other.y
        elif isinstance(other, (int, float)):
            x = self.x + other
            y = self.y + other
        return Vector(x, y)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return self.__sub__(other)

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            x = self.x * other.x
            y = self.y * other.y
            return x + y
        elif isinstance(other, (int, float)):
            x = self.x * other
            y = self.y * other
            return Vector(x, y)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            x = self.x / other
            y = self.y / other
        return Vector(x, y)

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __round__(self, ndigits=0):
        return Vector(round(self.x, ndigits), round(self.y, ndigits))

    def __str__(self):
        return f"({self.x}, {self.y})"


def fitParabola(p0: Vector, p1: Vector, p2: Vector) -> float:
    a = p0.x * (p0.y - p0.y) + p0.x * (p0.y - p0.y) + p0.x * (p0.y - p0.y)
    b = (
        p0.x * p0.x * (p0.y - p0.y)
        + p0.x * p0.x * (p0.y - p0.y)
        + p0.x * p0.x * (p0.y - p0.y)
    )
    return -b / (2 * a)


class Pose:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0):
        self.x = x
        self.y = y
        self.theta = theta

    def isColinear(self, other):
        if abs(self.theta - other.theta) > 0.001:
            return False
        diff = other - self
        angle = boundHalfRadians(np.arctan2(diff.y, diff.x))
        theta = boundHalfRadians(self.theta)
        return abs(angle - theta) < 0.001 or abs(angle - theta + np.pi) < 0.001

    def applyTwist(self, twist, dt):
        dsin = np.sin(twist.omega * dt) / twist.omega
        dcos = (np.cos(twist.omega * dt) - 1.0) / twist.omega
        sin = np.sin(self.theta)
        cos = np.cos(self.theta)
        dpose = Pose(
            twist.x * dsin + twist.y * dcos,
            twist.x * -dcos + twist.y * dsin,
            twist.omega * dt,
        )
        self.x += dpose.x * cos - dpose.y * sin
        self.y += dpose.x * sin - dpose.y * cos
        self.theta += dpose.theta

    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and (self.x == other.x)
            and (self.y == other.y)
            and (self.theta == other.theta)
        )

    def __add__(self, other):
        if isinstance(other, self.__class__):
            x = self.x + other.x
            y = self.y + other.y
            theta = self.theta + other.theta
        return Pose(x, y, theta)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return self.__sub__(other)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            x = self.x * other
            y = self.y * other
            theta = self.theta * other
        return Pose(x, y, theta)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            x = self.x / other
            y = self.y / other
            theta = self.theta / other
        return Pose(x, y, theta)

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __neg__(self):
        return Pose(-self.x, -self.y, -self.theta)

    def __round__(self, ndigits=0):
        return Pose(
            round(self.x, ndigits), round(self.y, ndigits), round(self.theta, ndigits)
        )

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta})"


class Twist:
    def __init__(self, x: float = 0, y: float = 0, omega: float = 0):
        self.x = x
        self.y = y
        self.omega = omega

    def applyTwist(self, twist, dt):
        dsin = np.sin(twist.omega * dt) / twist.omega
        dcos = (np.cos(twist.omega * dt) - 1.0) / twist.omega
        sin = np.sin(self.omega)
        cos = np.cos(self.omega)
        dpose = Twist(
            twist.x * dsin + twist.y * dcos,
            twist.x * -dcos + twist.y * dsin,
            twist.omega * dt,
        )
        self.x += dpose.x * cos - dpose.y * sin
        self.y += dpose.x * sin - dpose.y * cos
        self.omega += dpose.omega

    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and (self.x == other.x)
            and (self.y == other.y)
            and (self.omega == other.omega)
        )

    def __add__(self, other):
        if isinstance(other, self.__class__):
            x = self.x + other.x
            y = self.y + other.y
            omega = self.omega + other.omega
        return Twist(x, y, omega)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return self.__sub__(other)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            x = self.x * other
            y = self.y * other
            omega = self.omega * other
        return Twist(x, y, omega)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            x = self.x / other
            y = self.y / other
            omega = self.y / other
        return Twist(x, y, omega)

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __neg__(self):
        return Twist(-self.x, -self.y, -self.omega)

    def __round__(self, ndigits=0):
        return Twist(
            round(self.x, ndigits), round(self.y, ndigits), round(self.omega, ndigits)
        )

    def __str__(self):
        return f"({self.x}, {self.y}, {self.omega})"


class RobotState:
    def __init__(
        self,
        x: float = 0,
        y: float = 0,
        heading: float = 0,
        v: float = 0,
        omega: float = 0,
    ):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = v
        self.omega = omega

    def getPose(self):
        return Pose(self.x, self.y, self.heading)

    def setPose(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.heading = pose.theta

    def getTwist(self):
        return Twist(self.v, 0, self.omega)

    def setTwist(self, twist):
        self.v = twist.x
        self.om = twist.omega

    def update(self, last_state, dt: float) -> None:
        dx = self.x - last_state.x
        dy = self.y - last_state.y
        dheading = self.heading - last_state.heading
        self.v = np.hypot(dx, dy) / dt
        self.omega = dheading / dt

    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and (self.x == other.x)
            and (self.y == other.y)
            and (self.heading == other.heading)
            and (self.v == other.v)
            and (self.omega == other.omega)
        )

    def __add__(self, other):
        if isinstance(other, self.__class__):
            x = self.x + other.x
            y = self.y + other.y
            heading = self.heading + other.heading
            v = self.v + other.v
            omega = self.omega + other.omega
        return RobotState(x, y, heading, v, omega)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return self.__sub__(other)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            x = self.x * other
            y = self.y * other
            heading = self.heading * other
            v = self.v * other
            omega = self.omega * other
        return RobotState(x, y, heading, v, omega)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            x = self.x / other
            y = self.y / other
            heading = self.heading / other
            v = self.v / other
            omega = self.omega / other
        return RobotState(x, y, heading, v, omega)

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __neg__(self):
        return RobotState(-self.x, -self.y, -self.heading, -self.v, -self.omega)

    def __round__(self, ndigits=0):
        return RobotState(
            round(self.x, ndigits),
            round(self.y, ndigits),
            round(self.heading, ndigits),
            round(self.v, ndigits),
            round(self.omega, ndigits),
        )

    def __str__(self):
        return f"({self.x}, {self.y}, {self.heading}, {self.v}, {self.omega})"
