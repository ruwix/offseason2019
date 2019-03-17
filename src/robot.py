#!/usr/bin/env python
import magicbot
import wpilib
import ctre
from components.chassis import Chassis
from components.clickcounter import ClickCounter
from autonomous.trajectory import Trajectory
import numpy as np
from autonomous.ramsete import Ramsete


class Robot(magicbot.MagicRobot):
    chassis: Chassis
    clickcounter: ClickCounter

    VELOCITY_KP = 0.0
    VELOCITY_KI = 0.0
    VELOCITY_KD = 0.0
    VELOCITY_KF = 0.0
    KBETA = 2.0
    KZETA = 0.5

    def createObjects(self):
        np.set_printoptions(suppress=True)
        """Create motors and stuff here"""
        self.drive_motor_left = ctre.WPI_TalonSRX(1)
        self.drive_motor_right = ctre.WPI_TalonSRX(2)

        # for motor in (self.drive_motor_left, self.drive_motor_right):
        self.drive_motor_left.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.QuadEncoder, 0, 0
        )
        self.drive_motor_left.selectProfileSlot(0, 0)
        self.drive_motor_left.config_kP(0, self.VELOCITY_KP)
        self.drive_motor_left.config_kI(0, self.VELOCITY_KI)
        self.drive_motor_left.config_kD(0, self.VELOCITY_KD)
        self.drive_motor_left.config_kF(0, self.VELOCITY_KF)
        self.drive_motor_right.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.QuadEncoder, 0, 0
        )
        self.drive_motor_right.selectProfileSlot(0, 0)
        self.drive_motor_right.config_kP(0, self.VELOCITY_KP)
        self.drive_motor_right.config_kI(0, self.VELOCITY_KI)
        self.drive_motor_right.config_kD(0, self.VELOCITY_KD)
        self.drive_motor_right.config_kF(0, self.VELOCITY_KF)
        self.imu = ctre.PigeonIMU(3)

        self.driver = wpilib.joystick.Joystick(0)
        self.operator = wpilib.joystick.Joystick(1)
        self.timer = wpilib.Timer()

        self.ramsete = Ramsete(self.KBETA, self.KZETA)

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""
        try:
            self.chassis.setVelocityInput(
                -self.driver.getY(), self.driver.getThrottle()
            )
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
