from pyfrc.physics import drivetrains
from pyfrc.physics.units import units
from pyfrc import config

import numpy as np

from components.chassis import Chassis


class PhysicsEngine:
    def __init__(self, controller):
        self.controller = controller
        self.drive_max_velocity = Chassis.MAX_VELOCITY * 3.28084
        self.drivetrain = drivetrains.TwoMotorDrivetrain(speed=self.drive_max_velocity)
        self.quad_pos_left = 0
        self.quad_pos_right = 0

    def initialize(self, hal_data):
        hal_data.setdefault("custom", {})

    def update_sim(self, hal_data, now, dt):
        drive_left_talon = hal_data["CAN"][1]
        drive_right_talon = hal_data["CAN"][2]

        hal_data["custom"]["Pose"] = np.round(self.controller.get_position(), 2)
        hal_data["robot"]["pigeon_device_3"] = -np.degrees(
            self.controller.get_position()[2]
        )

        vl = drive_left_talon["value"]
        vr = drive_right_talon["value"]
        speed, rotation = self.drivetrain.get_vector(vl, -vr)
        self.controller.drive(speed, rotation, dt)

        quad_vel_left = Chassis.ENCODER_TICKS_PER_METER * self.drivetrain.l_speed
        quad_vel_right = Chassis.ENCODER_TICKS_PER_METER * self.drivetrain.r_speed
        self.quad_pos_left += quad_vel_left * dt
        self.quad_pos_right += quad_vel_right * dt
        drive_left_talon["quad_position"] = int(self.quad_pos_left)
        drive_right_talon["quad_position"] = int(self.quad_pos_right)
        drive_left_talon["quad_velocity"] = quad_vel_left
        drive_right_talon["quad_velocity"] = quad_vel_right
