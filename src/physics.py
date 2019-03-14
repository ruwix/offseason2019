from pyfrc.physics import drivetrains
from pyfrc.physics.units import units
from pyfrc import config

import numpy as np

from components.chassis import Chassis
from ctre import ControlMode


class DriveTrain:
    def __init__(self):
        self.vl = 0
        self.vr = 0
        self.x_wheelbase = Chassis.X_WHEELBASE / 12

    def getVelocities(self, vl: float, vr: float) -> np.array:
        vl /= 12
        vr /= 12
        v = (vl + vr) / 2
        omega = (vl - vr) / self.x_wheelbase
        return np.array([v, omega])


class SimulatedTalon:
    def __init__(self, id: int):
        self.id = id
        self.quad_pos = 0
        self.quad_vel = 0

    def getSpeed(self, hal_data: dict) -> None:
        talon = hal_data["CAN"][self.id]
        if talon["control_mode"] == ControlMode.PercentOutput:
            speed = talon["value"] * Chassis.MAX_VELOCITY
        elif talon["control_mode"] == ControlMode.Velocity:
            speed = talon["pid0_target"] / Chassis.ENCODER_TICKS_PER_INCH * 10
        return speed

    def update(self, hal_data: dict, dt: float) -> None:
        talon = hal_data["CAN"][self.id]
        self.quad_vel = self.getSpeed(hal_data) * Chassis.ENCODER_TICKS_PER_INCH
        self.quad_pos += self.quad_vel * dt
        talon["quad_position"] = int(self.quad_pos)
        talon["quad_velocity"] = int(self.quad_vel)


class PhysicsEngine:
    def __init__(self, controller):
        self.controller = controller
        self.drivetrain = DriveTrain()
        self.drive_left = SimulatedTalon(1)
        self.drive_right = SimulatedTalon(2)

    def initialize(self, hal_data):
        hal_data.setdefault("custom", {})

    def update_sim(self, hal_data, now, dt):
        pose = np.array(self.controller.get_position())
        pose[0:2] *= 12
        hal_data["custom"]["Pose"] = np.round(pose, 2)
        hal_data["robot"]["pigeon_device_3"] = np.rad2deg(pose[2])

        self.drive_left.update(hal_data, dt)
        self.drive_right.update(hal_data, dt)

        vl = self.drive_left.getSpeed(hal_data)
        vr = self.drive_right.getSpeed(hal_data)

        v, omega = self.drivetrain.getVelocities(vl, vr)

        self.controller.drive(v, omega, dt)

