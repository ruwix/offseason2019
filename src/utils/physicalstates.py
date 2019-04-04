class WheelState:
    """Can refer to velocity, acceleration, torque, voltage, etc., depending on context."""

    def __init__(self, left: float, right: float):
        self.left = left
        self.right = right


class ChassisState:
    """Can refer to velocity or acceleration depending on context."""

    def __init__(self, linear: float, angular: float):
        self.linear = linear
        self.angular = angular


class DriveDynamics:
    """Full state dynamics of the drivetrain."""

    def __init__(
        self,
        curvature: float,
        dcurvature: float,
        chassis_velocity: float,
        chassis_acceleration: float,
        wheel_velocity: float,
        wheel_acceleration: float,
        voltage: float,
        wheel_torque: float,
    ):
        self.curvature = curvature
        self.dcurvature = dcurvature
        self.chassis_velocity = chassis_velocity
        self.chassis_acceleration = chassis_acceleration
        self.wheel_velocity = wheel_velocity
        self.wheel_acceleration = wheel_acceleration
        self.voltage = voltage
        self.wheel_torque = wheel_torque
