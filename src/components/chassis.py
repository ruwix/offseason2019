import magicbot
import wpilib
import ctre
import numpy as np
from networktables import NetworkTables
from enum import Enum
from utils.geometry import RobotState, Twist
from copy import copy
from utils import units
from utils.pidf import PIDF


class Chassis:
    drive_motor_left: ctre.WPI_TalonSRX
    drive_motor_right: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU

    TRACK_WIDTH: float = 24 * units.meters_per_inch
    WHEELBASE: float = 24 * units.meters_per_inch

    WHEEL_DIAMETER: float = 6 * units.meters_per_inch
    WHEEL_CIRCUMFERENCE: float = np.pi * WHEEL_DIAMETER

    ENCODER_CPR: int = 4096
    ENCODER_GEAR_REDUCTION: int = 1

    ENCODER_TICKS_PER_METER: float = ENCODER_CPR * ENCODER_GEAR_REDUCTION / WHEEL_CIRCUMFERENCE
    MAX_VELOCITY: float = 3

    VELOCITY_KP = 0
    VELOCITY_KI = 0
    VELOCITY_KD = 0
    VELOCITY_KF = 2.0

    class _Mode(Enum):
        PercentOutput = 0
        Velocity = 1
        PercentVelocity = 2

    def __init__(self):
        self.timer = wpilib.Timer()
        self.left_pidf = PIDF(
            self.VELOCITY_KP, self.VELOCITY_KI, self.VELOCITY_KD, self.VELOCITY_KF
        )
        self.right_pidf = PIDF(
            self.VELOCITY_KP, self.VELOCITY_KI, self.VELOCITY_KD, self.VELOCITY_KF
        )

        self.l_signal = 0
        self.r_signal = 0
        self.mode = self._Mode.PercentOutput

        self.vl = 0
        self.vr = 0
        self._last_pl = 0
        self._last_pr = 0
        self.timestamp = 0
        self._last_timestamp = 0

        self.state = RobotState(0, 0, 0, 0, 0)
        self.last_state = self.state

        self._current_encoder_pos = 0
        self._last_encoder_pos = 0
        self._delta_encoder_pos = 0
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("Ariadne")

    def setWheelOutput(self, l_signal: float, r_signal: float) -> None:
        self.mode = self._Mode.PercentOutput
        self.l_signal = l_signal
        self.r_signal = r_signal

    def setWheelVelocity(self, l_signal: float, r_signal: float) -> None:
        self.mode = self._Mode.Velocity
        # if np.abs(vl) > self.MAX_VELOCITY or np.abs(vr) > self.MAX_VELOCITY:
        #     scale = self.MAX_VELOCITY / np.max((np.abs(vl), np.abs(vr)))
        #     vl *= scale
        #     vr *= scale
        self.l_signal = l_signal
        self.r_signal = r_signal

    def setChassisVelocity(self, v: float, omega: float) -> None:
        self.mode = self._Mode.Velocity
        l_signal = v + omega * self.TRACK_WIDTH / 2.0
        r_signal = v - omega * self.TRACK_WIDTH / 2.0
        self.setWheelVelocity(l_signal, r_signal)

    def setChassisTwist(self, twist: Twist) -> None:
        self.mode = self._Mode.Velocity
        self.setChassisVelocity(twist.x, twist.omega)

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
        self.l_signal = 0
        self.r_signal = 0
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
        pl = self.drive_motor_left.getSelectedSensorPosition(0)
        pr = self.drive_motor_right.getSelectedSensorPosition(0)

        self.vl = (pl - self._last_pl) / dt / self.ENCODER_TICKS_PER_METER
        self.vr = (pr - self._last_pr) / dt / self.ENCODER_TICKS_PER_METER
        if self.mode == self._Mode.PercentOutput:
            self.drive_motor_left.set(
                ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.l_signal
            )
            self.drive_motor_right.set(
                ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.r_signal
            )
        elif self.mode == self._Mode.Velocity:
            l_output = self.left_pidf.update(self.l_signal, self.vl, dt)
            r_output = self.right_pidf.update(self.r_signal, self.vr, dt)
            # print(f"{round(l_output,3)}\t{round(r_output,3)}")
            n=12
            l_output = np.clip(l_output, -n, n)
            r_output = np.clip(r_output, -n, n)
            print(f"{self.l_signal}\t{self.vl}")
            self.drive_motor_left.set(
                ctre.WPI_TalonSRX.ControlMode.PercentOutput, l_output/n
            )
            self.drive_motor_right.set(
                ctre.WPI_TalonSRX.ControlMode.PercentOutput, r_output/n
            )
        self._last_timestamp = self.timestamp
        self._last_pl = pl
        self._last_pr = pr
