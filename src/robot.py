#!/usr/bin/env python
import ctre
import magicbot
import numpy as np
import wpilib

from components.autoselector import AutoSelector
from components.chassis import Chassis
from controllers.ramsete import Ramsete
from utils.lazytalonsrx import LazyTalonSRX


class Robot(magicbot.MagicRobot):
    chassis: Chassis
    autoselector: AutoSelector
    VELOCITY_KP = 0.0
    VELOCITY_KI = 0.0
    VELOCITY_KD = 0.0
    VELOCITY_KF = 0.0
    KBETA = 20.0
    KZETA = 0.7

    def createObjects(self):
        np.set_printoptions(suppress=True)
        """Create motors and stuff here"""
        self.drive_motor_left = LazyTalonSRX(1)
        self.drive_motor_right = LazyTalonSRX(2)

        self.drive_motor_left.initialize(
            inverted=False, encoder=True, phase=False, name="Drive Motor Left"
        )
        self.drive_motor_right.initialize(
            inverted=False, encoder=True, phase=False, name="Drive Motor Right"
        )
        self.drive_motor_left.setPIDF(
            0, self.VELOCITY_KP, self.VELOCITY_KI, self.VELOCITY_KD, self.VELOCITY_KF
        )
        self.drive_motor_right.setPIDF(
            0, self.VELOCITY_KP, self.VELOCITY_KI, self.VELOCITY_KD, self.VELOCITY_KF
        )

        self.imu = ctre.PigeonIMU(3)

        self.driver = wpilib.joystick.Joystick(0)
        self.operator = wpilib.joystick.Joystick(1)

        self.ramsete = Ramsete(self.KBETA, self.KZETA)

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
            self.chassis.setPercentWheelVelocity(left, right)
            print(round(self.chassis.state, 3))
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
