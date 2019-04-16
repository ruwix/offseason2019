#!/usr/bin/env python
import ctre
import magicbot
import numpy as np
import wpilib

from components.autoselector import AutoSelector
from components.chassis import Chassis
from components.localization import Localization
from models.dcmotortransmission import DCMotorTransmission
from models.differentialdrive import DifferentialDrive
from utils import units
from utils.lazypigeonimu import LazyPigeonIMU
from utils.lazytalonsrx import LazyTalonSRX


class Robot(magicbot.MagicRobot):
    chassis: Chassis
    autoselector: AutoSelector
    localization: Localization

    VELOCITY_KP: float = 0.0
    VELOCITY_KI: float = 0.0
    VELOCITY_KD: float = 0.0
    VELOCITY_KF: float = 0.0
    MM_ACCEL = 1.0
    MM_VEL = 2.0

    ROBOT_LINEAR_INERTIA: float = 60
    ROBOT_ANGULAR_INERTIA: float = 80
    ROBOT_ANGUALR_DRAG: float = 12
    DRIVE_V_INTERCEPT: float = 1.055
    DRIVE_KV: float = 0.135
    DRIVE_KA: float = 0.012

    def createObjects(self):
        """Create motors and stuff here"""
        np.set_printoptions(suppress=True)
        self.dm_l = LazyTalonSRX(1)
        self.dm_r = LazyTalonSRX(2)

        self.dm_l.initialize(
            inverted=False, encoder=True, phase=False, name="Drive Motor Left"
        )
        self.dm_r.initialize(
            inverted=False, encoder=True, phase=False, name="Drive Motor Right"
        )
        self.dm_l.setPIDF(
            0, self.VELOCITY_KP, self.VELOCITY_KI, self.VELOCITY_KD, self.VELOCITY_KF
        )
        self.dm_r.setPIDF(
            0, self.VELOCITY_KP, self.VELOCITY_KI, self.VELOCITY_KD, self.VELOCITY_KF
        )

        self.dm_l.setMotionMagicConfig(self.MM_VEL, self.MM_ACCEL)
        self.dm_r.setMotionMagicConfig(self.MM_VEL, self.MM_ACCEL)

        self.imu = LazyPigeonIMU(3)

        self.driver = wpilib.joystick.Joystick(0)
        self.operator = wpilib.joystick.Joystick(1)

        self.transmission_l = DCMotorTransmission(
            1 / self.DRIVE_KV,
            Chassis.WHEEL_RADIUS ** 2 * self.ROBOT_LINEAR_INERTIA / (2 * self.DRIVE_KA),
            self.DRIVE_V_INTERCEPT,
        )
        self.transmission_r = self.transmission_l
        self.diff_drive = DifferentialDrive(
            self.ROBOT_LINEAR_INERTIA,
            self.ROBOT_ANGUALR_DRAG,
            self.ROBOT_ANGUALR_DRAG,
            Chassis.WHEEL_RADIUS,
            Chassis.TRACK_RADIUS,
            self.transmission_l,
            self.transmission_r,
        )

    def teleopInit(self):
        """Called when teleop starts; optional"""

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""
        try:
            speed = self.driver.getY()
            rotation = self.driver.getZ()
            left = speed + rotation
            right = speed - rotation
            if np.abs(left) < np.abs(right) and 1 < np.abs(right):
                left /= np.abs(right)
                right /= np.abs(right)
            if np.abs(right) < np.abs(left) and 1 < np.abs(left):
                left /= np.abs(left)
                right /= np.abs(left)
            self.chassis.setWheelOutput(left, right)
            print(round(self.chassis.state, 3))
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
