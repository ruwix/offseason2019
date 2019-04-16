from enum import Enum

import ctre
import magicbot
import numpy as np
import wpilib
from networktables import NetworkTables

from models.differentialdrive import DifferentialDrive
from utils import units
from utils.geometry import Pose
from utils.physicalstates import ChassisState


class Chassis:
    dm_l: ctre.WPI_TalonSRX
    dm_r: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU
    diff_drive: DifferentialDrive

    TRACK_WIDTH: float = 24 * units.meters_per_inch
    WHEELBASE: float = 24 * units.meters_per_inch
    TRACK_RADIUS: float = TRACK_WIDTH / 2

    ENCODER_CPR: int = 4096
    ENCODER_GEAR_REDUCTION: int = 1

    WHEEL_DIAMETER: float = 6 * units.meters_per_inch
    WHEEL_RADIUS: float = WHEEL_DIAMETER / 2
    WHEEL_CIRCUMFERENCE: float = np.pi * WHEEL_DIAMETER

    ENCODER_TICKS_PER_METER: float = ENCODER_CPR * ENCODER_GEAR_REDUCTION / WHEEL_CIRCUMFERENCE
    MAX_VELOCITY: float = 3

    class _Mode(Enum):
        PercentOutput = 0
        Velocity = 1

    def __init__(self):
        self.vl = 0
        self.vr = 0
        self.mode = self._Mode.PercentOutput

    def setWheelOutput(self, vl: float, vr: float) -> None:
        self.mode = self._Mode.PercentOutput
        self.vl = vl
        self.vr = vr

    def setWheelVelocity(self, vl: float, vr: float) -> None:
        self.mode = self._Mode.Velocity
        self.vl = int(vl * self.ENCODER_TICKS_PER_METER) / 10
        self.vr = int(vr * self.ENCODER_TICKS_PER_METER) / 10

    def setChassisVelocity(self, v: float, omega: float) -> None:
        self.setChassisState(ChassisState(v, omega))

    def setChassisState(self, velocity: ChassisState) -> None:
        self.mode = self._Mode.Velocity
        wheel_state = self.diff_drive.solveInverseKinematics(velocity)
        self.setWheelVelocity(
            wheel_state.left * self.WHEEL_RADIUS, wheel_state.right * self.WHEEL_RADIUS
        )

    def reset(self) -> None:
        self.vl = 0
        self.vr = 0

    def on_enable(self):
        """Called when the robot enters teleop or autonomous mode"""

    def execute(self):
        """Called periodically"""
        if self.mode == self._Mode.PercentOutput:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vr)
        elif self.mode == self._Mode.Velocity:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vr)
        elif self.mode == self._Mode.PercentVelocity:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vr)
