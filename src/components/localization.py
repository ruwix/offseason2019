from components.chassis import Chassis
import ctre
import wpilib
from networktables import NetworkTables
from utils.geometry import Pose
from models.differentialdrive import DifferentialDrive
import numpy as np


class Localization:
    dm_l: ctre.WPI_TalonSRX
    dm_r: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU
    diff_drive: DifferentialDrive
    chassis: Chassis

    def __init__(self):
        self.timer = wpilib.Timer()

        self.timestamp = 0
        self.last_timestamp = 0

        self.state = Pose()

        self.current_encoder_pos = 0
        self.last_encoder_pos = 0

        NetworkTables.initialize()
        self.table = NetworkTables.getTable("Ariadne")

    def setState(self, x, y, heading):
        self.state.x = x
        self.state.y = y
        self.state.theta = heading

    def updateState(self, dt: float) -> None:
        self.current_encoder_pos = (
            self.dm_l.getSelectedSensorPosition(0)
            + self.dm_r.getSelectedSensorPosition(0)
        ) / 2
        delta_encoder_pos = self.current_encoder_pos - self.last_encoder_pos

        self.state.theta = self.imu.getYaw()

        self.state.x += (
            np.cos(self.state.theta)
            * delta_encoder_pos
            / Chassis.ENCODER_TICKS_PER_METER
        )
        self.state.y += (
            np.sin(self.state.theta)
            * delta_encoder_pos
            / Chassis.ENCODER_TICKS_PER_METER
        )
        self.last_encoder_pos = self.current_encoder_pos

    def reset(self) -> None:
        self.timestamp = 0
        self.last_timestamp = 0
        self.state = Pose()

        self.current_encoder_pos = 0
        self.last_encoder_pos = 0
        self.timer.reset()

    def on_enable(self):
        self.timer.start()

    def execute(self):
        self.timestamp = self.timer.getFPGATimestamp()
        dt = self.timestamp - self.last_timestamp
        self.updateState(dt)
        self.table.putNumberArray(
            "Pose", np.array([self.state.x, self.state.y, self.state.theta])
        )
        self.last_timestamp = self.timestamp
