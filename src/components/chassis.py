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
        PercentVelocity = 2

    def __init__(self):
        self.timer = wpilib.Timer()

        self.vl = 0
        self.vr = 0
        self.mode = self._Mode.PercentOutput

        self.timestamp = 0
        self._last_timestamp = 0

        self.state = Pose()

        self._current_encoder_pos = 0
        self._last_encoder_pos = 0
        self._delta_encoder_pos = 0
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("Ariadne")

    def setWheelOutput(self, vl: float, vr: float) -> None:
        self.mode = self._Mode.PercentOutput
        self.vl = vl
        self.vr = vr

    def setWheelVelocity(self, vl: float, vr: float) -> None:
        self.mode = self._Mode.Velocity
        if np.abs(vl) > self.MAX_VELOCITY or np.abs(vr) > self.MAX_VELOCITY:
            scale = self.MAX_VELOCITY / np.max((np.abs(vl), np.abs(vr)))
            vl *= scale
            vr *= scale
        self.vl = int(vl * self.ENCODER_TICKS_PER_METER) / 10
        self.vr = int(vr * self.ENCODER_TICKS_PER_METER) / 10

    def setChassisVelocity(self, v: float, omega: float) -> None:
        self.mode = self._Mode.Velocity
        self.setChassisState(ChassisState(v, omega))

    def setChassisState(self, velocity: ChassisState) -> None:
        self.mode = self._Mode.Velocity
        wheel_state = self.diff_drive.solveInverseKinematics(velocity)
        self.setWheelVelocity(
            wheel_state.left * self.WHEEL_RADIUS, wheel_state.right * self.WHEEL_RADIUS
        )

    # def setChassisVelocity(self, v: float, omega: float) -> None:
    #     self.mode = self._Mode.Velocity
    #     vl = v - omega * self.TRACK_WIDTH / 2.0
    #     vr = v + omega * self.TRACK_WIDTH / 2.0
    #     self.setWheelVelocity(vl, vr)

    # def setChassisState(self, velocity: ChassisState) -> None:
    #     self.mode = self._Mode.Velocity
    #     self.setChassisVelocity(velocity.linear, velocity.angular)

    def setState(self, x, y, heading):
        self.state.x = x
        self.state.y = y
        self.state.theta = heading

    def updateState(self, dt: float) -> None:
        self._current_encoder_pos = (
            self.dm_l.getSelectedSensorPosition(0)
            + self.dm_r.getSelectedSensorPosition(0)
        ) / 2
        self._delta_encoder_pos = self._current_encoder_pos - self._last_encoder_pos

        self.state.theta = self.imu.getYaw()

        self.state.x += (
            np.cos(self.state.theta)
            * self._delta_encoder_pos
            / self.ENCODER_TICKS_PER_METER
        )
        self.state.y += (
            np.sin(self.state.theta)
            * self._delta_encoder_pos
            / self.ENCODER_TICKS_PER_METER
        )
        self._last_encoder_pos = self._current_encoder_pos

    def reset(self) -> None:
        self.vl = 0
        self.vr = 0
        self.timestamp = 0
        self._last_timestamp = 0
        self.state = Pose()

        self._current_encoder_pos = 0
        self._last_encoder_pos = 0
        self._delta_encoder_pos = 0
        self.timer.reset()

    def on_enable(self):
        """Called when the robot enters teleop or autonomous mode"""
        self.timer.start()

    def execute(self):
        """Called periodically"""
        self.timestamp = self.timer.getFPGATimestamp()
        dt = self.timestamp - self._last_timestamp
        self.updateState(dt)
        self.table.putNumberArray(
            "Pose", np.array([self.state.x, self.state.y, self.state.theta])
        )

        if self.mode == self._Mode.PercentOutput:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vr)
        elif self.mode == self._Mode.Velocity:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vr)
        elif self.mode == self._Mode.PercentVelocity:
            self.dm_l.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vl)
            self.dm_r.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vr)
        self._last_timestamp = self.timestamp
