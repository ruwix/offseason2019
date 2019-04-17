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
from utils.mathextension import getAngleDiff


class Chassis:
    dm_l: ctre.WPI_TalonSRX
    dm_r: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU
    diff_drive: DifferentialDrive

    TRACK_WIDTH: float = 24 * units.meters_per_inch  # m
    WHEELBASE: float = 24 * units.meters_per_inch  # m
    TRACK_RADIUS: float = TRACK_WIDTH / 2  # m

    ENCODER_CPR: int = 4096  # native units / revolution
    ENCODER_GEAR_REDUCTION: int = 1

    WHEEL_DIAMETER: float = 6 * units.meters_per_inch  # m
    WHEEL_RADIUS: float = WHEEL_DIAMETER / 2  # m
    WHEEL_CIRCUMFERENCE: float = np.pi * WHEEL_DIAMETER

    ENCODER_TICKS_PER_METER: float = ENCODER_CPR * ENCODER_GEAR_REDUCTION / WHEEL_CIRCUMFERENCE  # native units / m

    MAX_VELOCITY: float = 3  # m / s

    class _Mode(Enum):
        PercentOutput = 0
        Velocity = 1
        Position = 2

    def __init__(self):
        self.sl = 0
        self.sr = 0
        self.heading = 0
        self.mode = self._Mode.PercentOutput

    def setWheelOutput(self, sl: float, sr: float) -> None:
        self.mode = self._Mode.PercentOutput
        self.sl = sl
        self.sr = sr

    def setWheelVelocity(self, sl: float, sr: float) -> None:
        self.mode = self._Mode.Velocity
        self.sl = int(sl * self.ENCODER_TICKS_PER_METER) / 10
        self.sr = int(sr * self.ENCODER_TICKS_PER_METER) / 10

    def setChassisVelocity(self, v: float, omega: float) -> None:
        self.setChassisState(ChassisState(v, omega))

    def setChassisState(self, velocity: ChassisState) -> None:
        wheel_state = self.diff_drive.solveInverseKinematics(velocity)
        self.setWheelVelocity(
            wheel_state.left * self.WHEEL_RADIUS, wheel_state.right * self.WHEEL_RADIUS
        )

    def setWheelPosition(self, sl: float, sr: float):
        self.mode = self._Mode.Position
        self.sl = sl * self.ENCODER_TICKS_PER_METER
        self.sr = sr * self.ENCODER_TICKS_PER_METER

    def setHeading(self, heading: float):
        self.heading = heading
        error = self.TRACK_RADIUS * (getAngleDiff(self.imu.getYaw(), heading))
        sl = self.dm_l.getPosition() / self.ENCODER_TICKS_PER_METER + error
        sr = self.dm_r.getPosition() / self.ENCODER_TICKS_PER_METER - error
        self.setWheelPosition(sl, sr)

    def isAtHeading(self):
        return (
            self.mode == self._Mode.Position
            and abs(getAngleDiff(self.imu.getYaw(), self.heading))
            < 5 * units.radians_per_degree
        )

    def reset(self) -> None:
        self.sl = 0
        self.sr = 0
        self.heading = 0
        self.mode = self._Mode.PercentOutput

    def on_enable(self):
        """Called when the robot enters teleop or autonomous mode"""

    def execute(self):
        """Called periodically"""
        if self.mode == self._Mode.PercentOutput:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.sl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.sr)
        elif self.mode == self._Mode.Velocity:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.sl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.sr)
        elif self.mode == self._Mode.Position:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.MotionMagic, self.sl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.MotionMagic, self.sr)
