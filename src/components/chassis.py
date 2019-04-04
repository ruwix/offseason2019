import magicbot
import wpilib
import ctre
import numpy as np
from networktables import NetworkTables
from enum import Enum
from utils.geometry import RobotState
from utils.physicalstates import ChassisState
from copy import copy
from utils import units


class Chassis:
    drive_motor_left: ctre.WPI_TalonSRX
    drive_motor_right: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU

    X_WHEELBASE: float = 24 * units.meters_per_inch
    Y_WHEELBASE: float = 24 * units.meters_per_inch

    WHEEL_DIAMETER: float = 6 * units.meters_per_inch
    WHEEL_CIRCUMFERENCE: float = np.pi * WHEEL_DIAMETER

    ENCODER_CPR: int = 4096
    ENCODER_GEAR_REDUCTION: int = 1

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

        self.state = RobotState(0, 0, 0, 0, 0)
        self.last_state = self.state

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

    def setPercentWheelVelocity(self, vl: float, vr: float) -> None:
        self.mode = self._Mode.Velocity
        self.vl = int(vl * self.MAX_VELOCITY * self.ENCODER_TICKS_PER_METER / 10)
        self.vr = int(vr * self.MAX_VELOCITY * self.ENCODER_TICKS_PER_METER / 10)

    def setChassisVelocity(self, v: float, omega: float) -> None:
        self.mode = self._Mode.Velocity
        vl = v + omega * self.X_WHEELBASE / 2.0
        vr = v - omega * self.X_WHEELBASE / 2.0
        self.setWheelVelocity(vl, vr)

    def setChassisState(self, velocity: ChassisState) -> None:
        self.mode = self._Mode.Velocity
        self.setChassisVelocity(velocity.linear, velocity.angular)

    def setState(self, x, y, heading):
        self.state.x = x
        self.state.y = y
        self.state.heading = heading

    def updateState(self, dt: float) -> None:
        self._current_encoder_pos = (
            self.drive_motor_left.getSelectedSensorPosition(0)
            + self.drive_motor_right.getSelectedSensorPosition(0)
        ) / 2
        self._delta_encoder_pos = self._current_encoder_pos - self._last_encoder_pos

        self.state.heading = np.deg2rad(self.imu.getYawPitchRoll()[0])

        self.state.x += (
            np.cos(self.state.heading)
            * self._delta_encoder_pos
            / self.ENCODER_TICKS_PER_METER
        )
        self.state.y += (
            np.sin(self.state.heading)
            * self._delta_encoder_pos
            / self.ENCODER_TICKS_PER_METER
        )
        self.state.update(self.last_state, dt)
        self._last_encoder_pos = self._current_encoder_pos
        self.last_state = copy(self.state)

    def reset(self) -> None:
        self.vl = 0
        self.vr = 0
        self.timestamp = 0
        self._last_timestamp = 0
        self.state = RobotState(0, 0, 0, 0, 0)
        self.last_state = RobotState(0, 0, 0, 0, 0)

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
            "Pose", np.array([self.state.x, self.state.y, self.state.heading])
        )

        if self.mode == self._Mode.PercentOutput:
            self.drive_motor_left.set(
                ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vl
            )
            self.drive_motor_right.set(
                ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vr
            )
        elif self.mode == self._Mode.Velocity:
            self.drive_motor_left.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vl)
            self.drive_motor_right.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vr)
        elif self.mode == self._Mode.PercentVelocity:
            self.drive_motor_left.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vl)
            self.drive_motor_right.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vr)
        self._last_timestamp = self.timestamp
