import numpy as np
from ctre import ControlMode
from pyfrc import config

from components.chassis import Chassis
from utils import units
from utils.geometry import Vector

config.config_obj["pyfrc"]["robot"]["w"] = (
    config.config_obj["pyfrc"]["robot"]["w_m"] * units.feet_per_meter
)
config.config_obj["pyfrc"]["robot"]["h"] = (
    config.config_obj["pyfrc"]["robot"]["h_m"] * units.feet_per_meter
)
config.config_obj["pyfrc"]["robot"]["starting_x"] = (
    config.config_obj["pyfrc"]["robot"]["starting_x_m"] * units.feet_per_meter
)
config.config_obj["pyfrc"]["robot"]["starting_y"] = (
    config.config_obj["pyfrc"]["field"]["h_m"] / 2
    - config.config_obj["pyfrc"]["robot"]["starting_y_m"]
) * units.feet_per_meter
config.config_obj["pyfrc"]["field"]["w"] = int(
    config.config_obj["pyfrc"]["field"]["w_m"] * units.feet_per_meter
)
config.config_obj["pyfrc"]["field"]["h"] = int(
    config.config_obj["pyfrc"]["field"]["h_m"] * units.feet_per_meter
)
config.config_obj["pyfrc"]["field"]["px_per_ft"] = (
    config.config_obj["pyfrc"]["field"]["px_per_m"] * units.feet_per_meter
)


class DriveTrain:
    """A simplified drivetrain for robot simulation."""

    def __init__(self, track_width: float):
        self.track_width = track_width

    def getVelocities(self, vl: float, vr: float) -> np.array:
        """Get the linear/angular velocity of the robot given the wheel velocities."""
        v = (vl + vr) / 2
        omega = (vl - vr) / self.track_width
        return np.array([v, omega])


class PhysicsController:
    """A simplified physics controller for robot simulation that uses m/s."""

    def __init__(self, controller):
        self.controller = controller

    def drive(self, v: float, omega: float, dt: float) -> None:
        """Drive the robot given a linear velicty, angular velocity, and change in time."""
        self.controller.drive(v * units.feet_per_meter, omega, dt)

    def distanceDrive(self, x: float, y: float, theta: float) -> None:
        """Drive the robot forward x meters, right y meters, and turn theta radians."""
        self.controller.distance_drive(
            x * units.feet_per_meter, y * units.feet_per_meter, theta
        )

    def vectorDrive(self, vx: float, vy: float, omega: float, dt: float) -> None:
        """Drive the robot given an x velocity, a y velocity, and angular velocity, and change in time."""
        self.controller.drive(
            vx * units.feet_per_meter, vy * units.feet_per_meter, omega, dt
        )

    def getOffset(self, x: float, y: float) -> np.array:
        """Get the offset of the robot from a given positions."""
        offset = self.controller.get_offset(
            x * units.feet_per_meter, y * units.feet_per_meter
        )
        return np.array(
            [offset[0] * units.meters_per_foot, offset[1] * units.radians_per_degree]
        )

    def getPosition(self) -> np.array:
        """Get the position of the robot."""
        pos = self.controller.get_position()
        return np.array(
            [pos[0] * units.meters_per_foot, -pos[1] * units.meters_per_foot, pos[2]]
        )


class SimulatedDriveTalonSRX:
    """A simplified TalonSRX for simulation."""

    def __init__(self, id: int, max_velocity: float, ticks_per_meter: int):
        self.id = id
        self.max_velocity = max_velocity
        self.ticks_per_meter = ticks_per_meter
        self.quad_pos = 0
        self.quad_vel = 0

    def getVelocity(self, hal_data: dict) -> float:
        """Get the current velocity of the talon in m/s."""
        talon = hal_data["CAN"][self.id]
        if talon["control_mode"] == ControlMode.PercentOutput:
            velocity = talon["value"] * self.max_velocity
        elif talon["control_mode"] == ControlMode.Velocity:
            velocity = talon["pid0_target"] / self.ticks_per_meter * 10
        elif talon["control_mode"] == ControlMode.MotionMagic:
            error = talon["quad_position"] - talon["motionmagic_target"]
            if abs(error) < 1:
                velocity = 0
            if abs(error) < 10:
                velocity = 0.05 * -np.sign(error)
            elif abs(error) < 300:
                velocity = 0.2 * -np.sign(error)
            else:
                velocity = 1 * -np.sign(error)
        else:
            velocity = 0
        return velocity

    def update(self, hal_data: dict, dt: float) -> None:
        """Update the encoder position."""
        talon = hal_data["CAN"][self.id]
        self.quad_vel = self.getVelocity(hal_data) * self.ticks_per_meter
        self.quad_pos += self.quad_vel * dt
        talon["quad_position"] = int(self.quad_pos)
        talon["quad_velocity"] = int(self.quad_vel)


class PhysicsEngine:
    def __init__(self, controller):

        self.controller = PhysicsController(controller)
        self.drivetrain = DriveTrain(Chassis.TRACK_WIDTH)
        self.drive_left = SimulatedDriveTalonSRX(
            1, Chassis.MAX_VELOCITY, Chassis.ENCODER_TICKS_PER_METER
        )
        self.drive_right = SimulatedDriveTalonSRX(
            2, Chassis.MAX_VELOCITY, Chassis.ENCODER_TICKS_PER_METER
        )
        self.pose = self.controller.getPosition()
        self.last_pose = self.controller.getPosition()

    def initialize(self, hal_data):
        hal_data.setdefault("custom", {})

    def update_sim(self, hal_data, now, dt):
        self.pose = self.controller.getPosition()
        hal_data["custom"]["Pose"] = np.round(self.pose, 2)
        hal_data["custom"]["Velocity"] = np.round((self.last_pose - self.pose) / dt, 2)

        hal_data["robot"]["pigeon_device_3"] = -self.pose[2] * units.degrees_per_radian

        self.drive_left.update(hal_data, dt)
        self.drive_right.update(hal_data, dt)

        vl = self.drive_left.getVelocity(hal_data)
        vr = self.drive_right.getVelocity(hal_data)

        v, omega = self.drivetrain.getVelocities(vl, vr)

        self.controller.drive(v, omega, dt)
        self.last_pose = self.pose
