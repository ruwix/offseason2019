from enum import Enum

import ctre
import magicbot
import numpy as np
import wpilib
from networktables import NetworkTables

from models.differentialdrive import DifferentialDrive
from utils import units
from utils.geometry import Pose
from utils.mathextension import getAngleDiff
from utils.physicalstates import ChassisState, WheelState


class Chassis:
    dm_l: ctre.WPI_TalonSRX
    dm_r: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU
    diff_drive: DifferentialDrive

    WHEELBASE: float = 30 * units.meters_per_inch  # m
    TRACK_WIDTH: float = 30 * units.meters_per_inch  # m
    TRACK_RADIUS: float = TRACK_WIDTH / 2  # m

    ENCODER_CPR: int = 4096  # native units / revolution
    ENCODER_GEAR_REDUCTION: int = 1

    WHEEL_DIAMETER: float = 4 * units.meters_per_inch  # m
    WHEEL_RADIUS: float = WHEEL_DIAMETER / 2  # m
    WHEEL_CIRCUMFERENCE: float = np.pi * WHEEL_DIAMETER  # m

    ENCODER_TICKS_PER_METER: float = ENCODER_CPR * ENCODER_GEAR_REDUCTION / WHEEL_CIRCUMFERENCE  # native units / m

    MAX_VELOCITY: float = 3  # m / s

    class _Mode(Enum):
        PercentOutput = 0
        Velocity = 1
        Position = 2

    def __init__(self):
        self.signal = WheelState(0, 0)
        self.ff = WheelState(0, 0)
        self.heading = 0
        self.mode = self._Mode.PercentOutput

    def setWheelOutput(self, sl: float, sr: float) -> None:
        self.mode = self._Mode.PercentOutput
        self.signal = WheelState(sl, sr)

    def setWheelVelocity(self, velocity: WheelState, feedforward: WheelState) -> None:
        self.mode = self._Mode.Velocity
        self.signal.left = velocity.left
        self.signal.right = velocity.right
        self.ff = feedforward
        self.ff.left /= 12
        self.ff.right /= 12

    def setVelocityFromCurvature(self, velocity: float, curvature: float) -> None:
        self.setVelocityFromKinematics(ChassisState(velocity, velocity * curvature))

    def setVelocityFromKinematics(self, velocity: ChassisState) -> None:
        wheel_velocity = self.diff_drive.solveInverseKinematics(velocity)
        feedforward = self.diff_drive.getVoltagesFromkV(wheel_velocity)
        self.setWheelVelocity(wheel_velocity, feedforward)

    def setVelocityFromDynamics(
        self, chassis_velocity: ChassisState, chassis_acceleration: ChassisState
    ):
        dynamics = self.diff_drive.solveInverseChassisDynamics(
            chassis_velocity, chassis_acceleration
        )
        self.setWheelVelocity(dynamics.wheel_velocity, dynamics.voltage)

    def setVelocityFromTrajectoryTracker(self, output):
        self.setVelocityFromDynamics(output.velocity, output.acceleration)

    def setWheelPosition(self, sl: float, sr: float):
        self.mode = self._Mode.Position
        self.signal.left = sl
        self.signal.right = sr

    def setHeading(self, heading: float):
        self.heading = heading
        error = (
            self.diff_drive.track_radius
            * (getAngleDiff(self.imu.getYaw(), heading))
            / self.diff_drive.wheel_radius
        )
        sl = self.dm_l.getPosition() + error
        sr = self.dm_r.getPosition() - error
        self.setWheelPosition(sl, sr)

    def isAtHeading(self):
        return (
            self.mode == self._Mode.Position
            and abs(getAngleDiff(self.imu.getYaw(), self.heading))
            < 5 * units.radians_per_degree
        )

    def reset(self) -> None:
        self.signal = WheelState(0, 0)
        self.ff = WheelState(0, 0)
        self.heading = 0
        self.mode = self._Mode.PercentOutput

    def on_enable(self):
        """Called when the robot enters teleop or autonomous mode"""
        self.reset()

    def execute(self):
        """Called periodically"""
        if self.mode == self._Mode.PercentOutput:
            self.dm_l.setOutput(self.signal.left)
            self.dm_r.setOutput(self.signal.right)
        elif self.mode == self._Mode.Velocity:
            self.dm_l.setVelocity(self.signal.left, self.ff.left)
            self.dm_r.setVelocity(self.signal.right, self.ff.right)
        elif self.mode == self._Mode.Position:
            self.dm_l.setMotionMagicPosition(self.signal.left)
            self.dm_r.setMotionMagicPosition(self.signal.right)
