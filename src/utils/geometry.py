import numpy as np

from utils.epsilon import EPSILON
from utils.mathextension import lerp
from utils.physicalstates import ChassisState


def boundRadians(theta):
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

    def distance(self, other) -> float:
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

    def rotateBy(self, theta: float):
        """Get a vector that has been rotated about the origin by theta."""
        st, ct = np.sin(theta), np.cos(theta)
        x = (self.x * ct) - (self.y * st)
        y = (self.x * st) + (self.y * ct)
        return Vector(x, y)

    def lerp(self, other, t):
        return Vector(lerp(self.x, other.x, t), lerp(self.y, other.y, t))

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
        else:
            return NotImplemented
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
        else:
            return NotImplemented

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            x = self.x / other
            y = self.y / other
        else:
            return NotImplemented
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
        self.point = Vector(x, y)
        self.theta = theta

    def distance(self, other):
        return self.point.distance(other.point)

    def isColinear(self, other):
        if abs(self.theta - other.theta) > 0.001:
            return False
        diff = other - self
        angle = boundRadians(np.arctan2(diff.y, diff.x))
        theta = boundRadians(self.theta)
        return abs(angle - theta) < 0.001 or abs(angle - theta + np.pi) < 0.001

    def applyTwist(self, twist, dt):
        dsin = np.sin(twist.dtheta * dt) / twist.dtheta
        dcos = (np.cos(twist.dtheta * dt) - 1.0) / twist.dtheta
        sin = np.sin(self.theta)
        cos = np.cos(self.theta)
        dpose = Pose(
            twist.dx * dsin + twist.dy * dcos,
            twist.dx * -dcos + twist.dy * dsin,
            twist.dtheta * dt,
        )
        self.x += dpose.x * cos - dpose.y * sin
        self.y += dpose.x * sin - dpose.y * cos
        self.theta += dpose.theta

    def getTwist(self):
        dtheta = self.theta
        half_dtheta = dtheta / 2
        cos_minus_one = np.cos(self.theta) - 1.0

        if abs(cos_minus_one) < EPSILON:
            half_theta_by_tan_of_half_dtheta = 1 - 1 / 12 * dtheta ** 2
        else:
            half_theta_by_tan_of_half_dtheta = (
                -(half_dtheta * np.sin(self.theta)) / cos_minus_one
            )
        translation_part = Vector(self.x, self.y).rotateBy(
            np.arctan2(-half_dtheta, half_theta_by_tan_of_half_dtheta)
        )
        return Twist(translation_part.x, translation_part.y, self.theta)

    def transformBy(self, other):
        new_point = self.point + other.point.rotateBy(self.theta)
        return Pose(new_point.x, new_point.y, self.theta + other.theta)

    def inFrameOfReferenceOf(self, field_relative_origin):
        return (-field_relative_origin) + self

    def interpolate(self, other, t):
        if t <= 0:
            return self
        elif t >= 1:
            return other
        else:
            twist = (other - self).getTwist()
            return self + (twist * t).asPose()

    @property
    def x(self):
        return self.point.x

    @property
    def y(self):
        return self.point.y

    @x.setter
    def x(self, value):
        self.point.x = value

    @y.setter
    def y(self, value):
        self.point.y = value

    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and (self.x == other.x)
            and (self.y == other.y)
            and (self.theta == other.theta)
        )

    def __add__(self, other):
        if isinstance(other, self.__class__):
            return self.transformBy(other)
        else:
            return NotImplemented

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
        else:
            return NotImplemented
        return Pose(x, y, theta)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            x = self.x / other
            y = self.y / other
            theta = self.theta / other
        else:
            return NotImplemented
        return Pose(x, y, theta)

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __neg__(self):
        new_point = (-self.point).rotateBy(-self.theta)
        return Pose(new_point.x, new_point.y, -self.theta)

    def __round__(self, ndigits=0):
        return Pose(
            round(self.x, ndigits), round(self.y, ndigits), round(self.theta, ndigits)
        )

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta})"


class PoseWithCurvature:
    def __init__(
        self,
        x: float = 0,
        y: float = 0,
        theta: float = 0,
        curvature: float = 0,
        dkds: float = 0,
    ):
        self.pose = Pose(x, y, theta)
        self.curvature = curvature
        self.dkds = dkds

    def distance(self, other):
        return self.pose.distance(other.pose)

    def interpolate(self, other, t):
        if t <= 0:
            return self
        elif t >= 1:
            return other
        else:
            interpolated_pose = self.pose.interpolate(other.pose, t)
            return PoseWithCurvature(
                interpolated_pose.x,
                interpolated_pose.y,
                interpolated_pose.theta,
                lerp(self.curvature, other.curvature, t),
                lerp(self.dkds, other.dkds, t),
            )

    @property
    def x(self):
        return self.pose.x

    @property
    def y(self):
        return self.pose.y

    @property
    def theta(self):
        return self.pose.theta

    @x.setter
    def x(self, value):
        self.pose.x = value

    @y.setter
    def y(self, value):
        self.pose.y = y

    @theta.setter
    def theta(self, value):
        self.pose.theta = value

    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and (self.pose == other.pose)
            and (self.curvature == other.curvature)
            and (self.dkds == other.dkds)
        )

    def __add__(self, other):
        if isinstance(other, self.__class__):
            pose = self.pose + other.pose
            curvature = self.curvature + other.curvature
            dkds = self.dkds + other.dkds
            return PoseWithCurvature(pose.x, pose.y, pose.theta, curvature, dkds)
        else:
            return NotImplemented

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return self.__sub__(other)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            pose = self.pose / other
            curvature = self.curvature * other
            dkds = self.dkds * other
        else: 
            return NotImplemented
        return PoseWithCurvature(pose.x, pose.y, pose.theta, curvature, dkds)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            pose = self.pose / other
            curvature = self.curvature / other
            dkds = self.dkds / other
        else: 
            return NotImplemented
        return PoseWithCurvature(pose.x, pose.y, pose.theta, curvature, dkds)

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __neg__(self):
        return PoseWithCurvature(
            -self.x, -self.y, -self.theta, -self.curvature, -self.dkds
        )

    def __round__(self, ndigits=0):
        return PoseWithCurvature(
            round(self.x, ndigits),
            round(self.y, ndigits),
            round(self.theta, ndigits),
            round(self.curvature, ndigits),
            round(self.dkds, ndigits),
        )

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta}, {self.curvature}, {self.dkds})"


class Twist:
    def __init__(self, dx: float = 0, dy: float = 0, dtheta: float = 0):
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta

    def applyTwist(self, twist, dt):
        dsin = np.sin(twist.dtheta * dt) / twist.dtheta
        dcos = (np.cos(twist.dtheta * dt) - 1.0) / twist.dtheta
        sin = np.sin(self.dtheta)
        cos = np.cos(self.dtheta)
        dpose = Twist(
            twist.dx * dsin + twist.dy * dcos,
            twist.dx * -dcos + twist.dy * dsin,
            twist.dtheta * dt,
        )
        self.dx += dpose.dx * cos - dpose.dy * sin
        self.dy += dpose.dx * sin - dpose.dy * cos
        self.dtheta += dpose.dtheta

    def asPose(self):
        sin_theta = np.sin(self.dtheta)
        cos_theta = np.cos(self.dtheta)
        if abs(self.dtheta < EPSILON):
            s = 1 - 1 / 6 * self.dtheta ** 2
            c = self.dtheta / 2
        else:
            s = sin_theta / self.dtheta
            c = (1 - cos_theta) / self.dtheta
        return Pose(self.dx * s - self.dy * c, self.dx * c + self.dy * s, self.dtheta)

    def lerp(self, other, t):
        return Pose(
            lerp(self.dx, other.dx, t),
            lerp(self.dy, other.dy, t),
            lerp(self.dtheta, other.dtheta, t),
        )

    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and (self.dx == other.dx)
            and (self.dy == other.dy)
            and (self.dtheta == other.dtheta)
        )

    def __add__(self, other):
        if isinstance(other, self.__class__):
            dx = self.dx + other.dx
            dy = self.dy + other.dy
            dtheta = self.dtheta + other.dtheta
        else: 
            return NotImplemented
        return Twist(dx, dy, dtheta)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return self.__sub__(other)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            dx = self.dx * other
            dy = self.dy * other
            dtheta = self.dtheta * other
        else: 
            return NotImplemented
        return Twist(dx, dy, dtheta)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            dx = self.dx / other
            dy = self.dy / other
            dtheta = self.dy / other
        else: 
            return NotImplemented
        return Twist(dx, dy, dtheta)

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __neg__(self):
        return Twist(-self.dx, -self.dy, -self.dtheta)

    def __round__(self, ndigits=0):
        return Twist(
            round(self.dx, ndigits),
            round(self.dy, ndigits),
            round(self.dtheta, ndigits),
        )

    def __str__(self):
        return f"({self.dx}, {self.dy}, {self.dtheta})"


class RobotState:
    def __init__(
        self,
        x: float = 0,
        y: float = 0,
        heading: float = 0,
        v: float = 0,
        omega: float = 0,
        a: float = 0,
        alpha: float = 0,
    ):
        self.pose = Pose(x, y, heading)
        self.velocity = ChassisState(v, omega)
        self.acceleration = ChassisState(a, alpha)

    @property
    def x(self):
        return self.pose.x

    @property
    def y(self):
        return self.pose.y

    @property
    def heading(self):
        return self.pose.theta

    @property
    def v(self):
        return self.velocity.linear

    @property
    def omega(self):
        return self.velocity.angular

    @property
    def a(self):
        return self.acceleration.linear

    @property
    def alpha(self):
        return self.acceleration.angular

    @x.setter
    def x(self, value):
        self.pose.x = value

    @y.setter
    def y(self, value):
        self.pose.y = value

    @heading.setter
    def heading(self, value):
        self.pose.theta = value

    @v.setter
    def v(self, value):
        self.velocity.linear = value

    @omega.setter
    def omega(self, value):
        self.velocity.angular = value

    @a.setter
    def a(self, value):
        self.acceleration.linear = value

    @alpha.setter
    def alpha(self, value):
        self.acceleration.angular = value

    def update(self, last_state, dt: float) -> None:
        dstate = self - last_state
        direction = (
            1
            if np.sign(np.arctan2(dstate.y, dstate.x))
            == np.sign(boundRadians(dstate.heading))
            else -1
        )
        self.v = direction * np.hypot(dstate.x, dstate.y) / dt
        self.omega = dstate.heading / dt
        dstate = self - last_state
        direction = np.sign(dstate.v)
        self.a = dstate.v / dt
        self.alpha = dstate.omega / dt

    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and (self.x == other.x)
            and (self.y == other.y)
            and (self.heading == other.heading)
            and (self.v == other.v)
            and (self.omega == other.omega)
            and (self.a == other.a)
            and (self.alpha == other.alpha)
        )

    def __add__(self, other):
        if isinstance(other, self.__class__):
            x = self.x + other.x
            y = self.y + other.y
            heading = self.heading + other.heading
            v = self.v + other.v
            omega = self.omega + other.omega
            a = self.a + other.a
            alpha = self.alpha + other.alpha
        else: 
            return NotImplemented
        return RobotState(x, y, heading, v, omega, a, alpha)

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
            a = self.a * other.a
            alpha = self.alpha * other.alpha
        else: 
            return NotImplemented
        return RobotState(x, y, heading, v, omega, a, alpha)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            x = self.x / other
            y = self.y / other
            heading = self.heading / other
            v = self.v / other
            omega = self.omega / other
            a = self.a / other.a
            alpha = self.alpha / other.alpha
        else: 
            return NotImplemented
        return RobotState(x, y, heading, v, omega, a, alpha)

    def __rtruediv__(self, other):
        return self.__truediv__(other)

    def __neg__(self):
        return RobotState(
            -self.x, -self.y, -self.heading, -self.v, -self.omega, -self.a, -self.alpha
        )

    def __round__(self, ndigits=0):
        return RobotState(
            round(self.x, ndigits),
            round(self.y, ndigits),
            round(self.heading, ndigits),
            round(self.v, ndigits),
            round(self.omega, ndigits),
            round(self.a, ndigits),
            round(self.alpha, ndigits),
        )

    def __str__(self):
        return f"({self.x},\t{self.y},\t{self.heading},\t{self.v},\t{self.omega},\t{self.a},\t{self.alpha})"
