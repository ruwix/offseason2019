import numpy as np

from components.chassis import Chassis
from ctre import ControlMode
from utils import units


class DriveTrain:
    def __init__(self, x_wheelbase: float):
        self.x_wheelbase = x_wheelbase

    def getVelocities(self, vl: float, vr: float) -> np.array:
        v = (vl + vr) / 2
        omega = (vl - vr) / self.x_wheelbase
        return np.array([v, omega])


class PhysicsController:
    def __init__(self, controller):
        self.controller = controller

    def drive(self, v: float, omega: float, dt: float) -> None:
        self.controller.drive(v * units.feet_per_meter, omega, dt)

    def distanceDrive(self, x: float, y: float, angle: float) -> None:
        self.controller.distance_drive(
            x * units.feet_per_meter, y * units.feet_per_meter, angle
        )

    def vectorDrive(self, vx: float, vy: float, omega: float, dt: float) -> None:
        self.controller.drive(
            vx * units.feet_per_meter, vy * units.feet_per_meter, omega, dt
        )

    def getOffset(self, x: float, y: float) -> np.array:
        offset = self.controller.get_offset(
            x * units.feet_per_meter, y * units.feet_per_meter
        )
        return np.array([offset[0] * units.meters_per_foot, np.deg2rad(offset[1])])

    def getPosition(self) -> np.array:
        pos = self.controller.get_position()
        return np.array(
            [pos[0] * units.meters_per_foot, pos[1] * units.meters_per_foot, pos[2]]
        )


class SimulatedDriveTalon:
    def __init__(self, id: int, max_velocity: float, ticks_per_meter: int):
        self.id = id
        self.max_velocity = max_velocity
        self.ticks_per_meter = ticks_per_meter
        self.quad_pos = 0
        self.quad_vel = 0

    def getVelocity(self, hal_data: dict) -> float:
        talon = hal_data["CAN"][self.id]
        if talon["control_mode"] == ControlMode.PercentOutput:
            velocity = talon["value"] * self.max_velocity
        elif talon["control_mode"] == ControlMode.Velocity:
            velocity = talon["pid0_target"] / self.ticks_per_meter * 10
        return velocity

    def update(self, hal_data: dict, dt: float) -> None:
        talon = hal_data["CAN"][self.id]
        self.quad_vel = self.getVelocity(hal_data) * self.ticks_per_meter
        self.quad_pos += self.quad_vel * dt
        talon["quad_position"] = int(self.quad_pos)
        talon["quad_velocity"] = int(self.quad_vel)


class PhysicsEngine:
    def __init__(self, controller):
        self.controller = PhysicsController(controller)
        self.drivetrain = DriveTrain(Chassis.X_WHEELBASE)
        self.drive_left = SimulatedDriveTalon(
            1, Chassis.MAX_VELOCITY, Chassis.ENCODER_TICKS_PER_METER
        )
        self.drive_right = SimulatedDriveTalon(
            2, Chassis.MAX_VELOCITY, Chassis.ENCODER_TICKS_PER_METER
        )

    def initialize(self, hal_data):
        hal_data.setdefault("custom", {})

    def update_sim(self, hal_data, now, dt):
        pose = self.controller.getPosition()
        hal_data["custom"]["Pose"] = np.round(pose, 2)
        hal_data["robot"]["pigeon_device_3"] = np.rad2deg(pose[2])

        self.drive_left.update(hal_data, dt)
        self.drive_right.update(hal_data, dt)

        vl = self.drive_left.getVelocity(hal_data)
        vr = self.drive_right.getVelocity(hal_data)

        v, omega = self.drivetrain.getVelocities(vl, vr)

        self.controller.drive(v, omega, dt)

