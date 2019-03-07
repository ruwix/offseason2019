import magicbot
import wpilib
import ctre
import numpy as np
from pint import UnitRegistry


class Chassis:
    drive_motor_left: ctre.WPI_TalonSRX
    drive_motor_right: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU

    units = UnitRegistry()
    X_WHEELBASE: float = 0.5
    Y_WHEELBASE: float = 0.5

    WHEEL_DIAMETER: float = 0.1524
    WHEEL_CIRCUMFERENCE: float = np.pi * WHEEL_DIAMETER

    ENCODER_CPR: int = 4096
    ENCODER_GEAR_REDUCTION: int = 1

    ENCODER_TICKS_PER_METER = ENCODER_CPR * ENCODER_GEAR_REDUCTION / WHEEL_CIRCUMFERENCE
    MAX_VELOCITY: float = 2

    def __init__(self):
        self.timer = wpilib.Timer()

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

    def setOutput(self, _vl, _vr):
        self.vl = _vl
        self.vr = _vr

    def setInput(self, speed, rotation):
        self.vl = speed + rotation
        self.vr = speed - rotation

    def updateOdometry(self, dt):
        self._current_encoder_pos = (
            self.drive_motor_left.getSelectedSensorPosition(0)
            + self.drive_motor_right.getSelectedSensorPosition(0)
        ) / 2
        self._delta_encoder_pos = self._current_encoder_pos - self._last_encoder_pos

        self.odometry[2] = self.imu.getYawPitchRoll()[0] % 360
        theta_radians = np.deg2rad(self.odometry[2])
        self.odometry[0] += (
            np.cos(theta_radians)
            * self._delta_encoder_pos
            / self.ENCODER_TICKS_PER_METER
        )
        self.odometry[1] -= (
            np.sin(theta_radians)
            * self._delta_encoder_pos
            / self.ENCODER_TICKS_PER_METER
        )

        self.velocity = (self.odometry - self._last_odometry) / dt
        # self.acceleration = self.velocity - self._last_velocity

        self._last_encoder_pos = self._current_encoder_pos
        self._last_odometry = self.odometry
        # self._last_velocity = self.velocity

    def setOdometry(self, x, y, theta):
        self.odometry = np.array([x, y, theta])

    def getOdometry(self):
        return self.odometry

    def reset(self):
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
        self.drive_motor_left.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vl)
        self.drive_motor_right.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vr)
        self._last_timestamp = self.timestamp
