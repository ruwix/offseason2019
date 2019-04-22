import ctre
import numpy as np
import wpilib
from networktables import NetworkTables

from components.chassis import Chassis
from models.differentialdrive import DifferentialDrive
from utils.geometry import Pose, Twist


class Localization:
    dm_l: ctre.WPI_TalonSRX
    dm_r: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU
    diff_drive: DifferentialDrive
    chassis: Chassis

    def __init__(self):
        self.state = Pose()

        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.last_heading = 0

        NetworkTables.initialize()
        self.table = NetworkTables.getTable("Ariadne")

    def setState(self, x, y, heading):
        self.state.x = x
        self.state.y = y
        self.state.theta = heading

    def update(self) -> None:
        cur_left_encoder = self.dm_l.getPosition()
        cur_right_encoder = self.dm_r.getPosition()
        cur_heading = self.imu.getYaw()

        delta_left = cur_left_encoder - self.last_left_encoder
        delta_right = cur_right_encoder - self.last_right_encoder

        dx = (delta_left + delta_right) / 2 / Chassis.ENCODER_TICKS_PER_METER
        dheading = cur_heading - self.last_heading

        self.state += Twist(dx, 0, dheading).asPose()

        self.last_left_encoder = cur_left_encoder
        self.last_right_encoder = cur_right_encoder
        self.last_heading = cur_heading

    def reset(self) -> None:
        self.state = Pose()
        self.current_encoder_pos = 0
        self.last_encoder_pos = 0

    def on_enable(self):
        pass

    def execute(self):
        self.update()
        self.table.putNumberArray(
            "Pose", np.array([self.state.x, self.state.y, self.state.theta])
        )
