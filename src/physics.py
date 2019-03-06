from pyfrc.physics import drivetrains

from pyfrc import config
from components.chassis import Chassis


class PhysicsEngine:
    def __init__(self, controller):
        self.controller = controller
        self.drivetrain = drivetrains.TwoMotorDrivetrain()

    def initialize(self, hal_data):
        hal_data.setdefault("custom", {})

    def update_sim(self, hal_data, now, tm_diff):
        drive_left_talon = hal_data["CAN"][1]
        drive_right_talon = hal_data["CAN"][2]

        hal_data["custom"]["Pose"] = [
            round(i, 2) for i in self.controller.get_position()
        ]
        hal_data["robot"]["pigeon_device_3"] = self.controller.get_position()[2]
        vl = drive_left_talon["value"]
        vr = drive_right_talon["value"]
        speed, rotation = self.drivetrain.get_vector(vl, -vr)
        self.controller.drive(speed, rotation, tm_diff)
