import magicbot
import wpilib
import ctre
import numpy as np
from networktables import NetworkTables
from enum import Enum


class Chassis:
    drive_motor_left: ctre.WPI_TalonSRX
    drive_motor_right: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU

    X_WHEELBASE: float = 24
    Y_WHEELBASE: float = 24

    WHEEL_DIAMETER: float = 6
    WHEEL_CIRCUMFERENCE: float = np.pi * WHEEL_DIAMETER

    ENCODER_CPR: int = 4096
    ENCODER_GEAR_REDUCTION: int = 1

    ENCODER_TICKS_PER_INCH: float = ENCODER_CPR * ENCODER_GEAR_REDUCTION / WHEEL_CIRCUMFERENCE
    MAX_VELOCITY: float = 120

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

        self.odometry = np.empty(3)
        self._last_odometry = np.empty(3)

        self.velocity = np.empty(2)

        self._current_encoder_pos = 0
        self._last_encoder_pos = 0
        self._delta_encoder_pos = 0
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("RobotOdyssey")

    def setOutput(self, vl: float, vr: float) -> None:
        self.mode = self._Mode.PercentOutput
        self.vl = vl
        self.vr = vr

    def setInput(self, speed: float, rotation: float) -> None:
        self.mode = self._Mode.PercentOutput
        self.vl = speed + rotation
        self.vr = speed - rotation

    def setVelocityInput(self, speed: float, rotation: float) -> None:
        self.mode = self._Mode.Velocity
        vl = speed + rotation
        vr = speed - rotation
        if np.abs(vl) > 1 or np.abs(vr) > 1:
            scale = np.abs(np.max((vl, vr)))
            vl /= scale
            vr /= scale
        self.setPercentVelocity(vl, vr)

    def setVelocity(self, vl: float, vr: float) -> None:
        self.mode = self._Mode.Velocity
        # if np.abs(vl) > self.MAX_VELOCITY or np.abs(vr) > self.MAX_VELOCITY:
        #     scale = self.MAX_VELOCITY / np.max((np.abs(vl), np.abs(vr)))
        #     vl *= scale
        #     vr *= scale
        self.vl = int(vl * self.ENCODER_TICKS_PER_INCH) / 10
        self.vr = int(vr * self.ENCODER_TICKS_PER_INCH) / 10

    def setPercentVelocity(self, vl: float, vr: float) -> None:
        self.mode = self._Mode.Velocity
        self.vl = int(vl * self.MAX_VELOCITY * self.ENCODER_TICKS_PER_INCH / 10)
        self.vr = int(vr * self.MAX_VELOCITY * self.ENCODER_TICKS_PER_INCH / 10)

    def updateOdometry(self, dt: float) -> None:
        self._current_encoder_pos = (
            self.drive_motor_left.getSelectedSensorPosition(0)
            + self.drive_motor_right.getSelectedSensorPosition(0)
        ) / 2
        self._delta_encoder_pos = self._current_encoder_pos - self._last_encoder_pos

        self.odometry[2] = self.imu.getYawPitchRoll()[0]
        theta_radians = np.deg2rad(self.odometry[2])
        self.odometry[0] += (
            np.cos(theta_radians)
            * self._delta_encoder_pos
            / self.ENCODER_TICKS_PER_INCH
        )
        self.odometry[1] -= (
            np.sin(theta_radians)
            * self._delta_encoder_pos
            / self.ENCODER_TICKS_PER_INCH
        )

        velocity_all = (self.odometry - self._last_odometry) / dt
        self.velocity[0] = np.hypot(velocity_all[0], velocity_all[1])
        self.velocity[1] = velocity_all[2]
        self._last_encoder_pos = self._current_encoder_pos
        self._last_odometry = self.odometry

    def setOdometry(self, x: float, y: float, theta: float) -> None:
        self.odometry = np.array([x, y, theta])

    def resetOdometry(self) -> None:
        self.odometry = np.zeros(3)

    def getWheelVelocities(self, v: float, omega: float) -> np.array:
        scale = 1
        if np.abs(v) > self.MAX_VELOCITY:
            scale = self.MAX_VELOCITY / v
        v *= scale
        omega *= scale
        left = v - np.deg2rad(omega) * self.X_WHEELBASE / 2.0
        right = v + np.deg2rad(omega) * self.X_WHEELBASE / 2.0
        return np.array([left, right])

    def reset(self) -> None:
        self.vl = 0
        self.vr = 0
        self.timestamp = 0
        self._last_timestamp = 0
        self.odometry = np.zeros(3)
        self._last_odometry = np.zeros(3)
        self.velocity = np.zeros(3)
        self._last_velocity = np.zeros(3)
        self.acceleration = np.zeros(3)
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
        self.updateOdometry(dt)
        self.table.putNumberArray("Pose", self.odometry)

        if self.mode == self._Mode.PercentOutput:
            self.drive_motor_left.set(
                ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vl
            )
            self.drive_motor_right.set(
                ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vr
            )
        elif self.mode == self._Mode.Velocity:
            # print(f"{self.vl} - {self.vr}")
            self.drive_motor_left.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vl)
            self.drive_motor_right.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vr)
        elif self.mode == self._Mode.PercentVelocity:
            self.drive_motor_left.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vl)
            self.drive_motor_right.set(ctre.WPI_TalonSRX.ControlMode.Velocity, self.vr)
        self._last_timestamp = self.timestamp

