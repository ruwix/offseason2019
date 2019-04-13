#!/usr/bin/env python
import ctre
import magicbot
import numpy as np
import wpilib

from components.autoselector import AutoSelector
from components.chassis import Chassis
from utils.lazytalonsrx import LazyTalonSRX


class Robot(magicbot.MagicRobot):
    chassis: Chassis
    autoselector: AutoSelector

    VELOCITY_KP = 0.0
    VELOCITY_KI = 0.0
    VELOCITY_KD = 0.0
    VELOCITY_KF = 0.0
    # TODO add differential drive object

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

        self.imu = ctre.PigeonIMU(3)

        self.driver = wpilib.joystick.Joystick(0)
        self.operator = wpilib.joystick.Joystick(1)

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
