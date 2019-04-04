import numpy as np

from components.chassis import Chassis
from ctre import ControlMode
from utils import units
from utils.geometry import Vector
from utils.geometry import Vector
from utils.realdifferentialdrive import RealDifferentialDrive
from utils.realdcmotor import RealDCMotor


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
        return np.array([offset[0] * units.meters_per_foot, np.deg2rad(offset[1])])

    def getPosition(self) -> np.array:
        """Get the position of the robot."""
        pos = self.controller.get_position()
        return np.array(
            [pos[0] * units.meters_per_foot, pos[1] * units.meters_per_foot, pos[2]]
        )


class SimulatedDriveTalonSRX:
    """A simplified TalonSRX for simulation."""

    def __init__(self, id: int, max_velocity: float, ticks_per_meter: int):
        self.id = id
        self.max_velocity = max_velocity
        self.ticks_per_meter = ticks_per_meter
        self.quad_pos = 0
        self.quad_vel = 0

    def getVoltage(self, hal_data: dict):
        talon = hal_data["CAN"][self.id]
        if talon["control_mode"] == ControlMode.PercentOutput:
            return talon["value"] * 12
        else:
            return 0

    def getVelocity(self, hal_data: dict) -> float:
        """Get the current velocity of the talon in m/s."""
        talon = hal_data["CAN"][self.id]
        if talon["control_mode"] == ControlMode.PercentOutput:
            velocity = talon["value"] * self.max_velocity
        elif talon["control_mode"] == ControlMode.Velocity:
            velocity = talon["pid0_target"] / self.ticks_per_meter * 10
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
        self.motor = RealDCMotor(12.0, 2.42, 133.0, 2.7, 5310.0)
        self.gearbox = RealDCMotor.gearbox(self.motor, 2)
        self.drive = RealDifferentialDrive(
            gearbox=self.gearbox,
            gear_ratio=5,
            track_width=Chassis.TRACK_WIDTH / 2,
            wheel_radius=Chassis.WHEEL_DIAMETER / 2,
            mass=50,
            moi=10,
        )
        self.controller = PhysicsController(controller)
        self.drivetrain = DriveTrain(Chassis.TRACK_WIDTH / 2)
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

        hal_data["robot"]["pigeon_device_3"] = np.rad2deg(self.pose[2])

        self.drive_left.update(hal_data, dt)
        self.drive_right.update(hal_data, dt)
        vl = self.drive_left.getVoltage(hal_data)
        vr = self.drive_right.getVoltage(hal_data)
        self.drive.update(vl, vr, dt)
        v, omega = self.drivetrain.getVelocities(self.drive.vl, self.drive.vr)
        self.controller.drive(v, omega, dt)
        self.last_pose = self.pose
