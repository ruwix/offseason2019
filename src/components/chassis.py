import magicbot
import wpilib
import ctre
import numpy as np


class Chassis:
    drive_motor_left: ctre.WPI_TalonSRX
    drive_motor_right: ctre.WPI_TalonSRX
    imu: ctre.PigeonIMU

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

        self.odometry[2] = self.imu.getYawPitchRoll()[0] % (2 * np.pi)
        self.odometry[0] += np.cos(self.odometry[2]) * self._delta_encoder_pos
        self.odometry[1] += np.sin(self.odometry[2]) * self._delta_encoder_pos

        self.velocity = self.odometry - self._last_odometry
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
        self.drive_motor_left.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vl)
        self.drive_motor_right.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, self.vr)
        self.updateOdometry(dt)
        self._last_timestamp = self.timestamp
