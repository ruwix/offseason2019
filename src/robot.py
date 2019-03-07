#!/usr/bin/env python
import magicbot
import wpilib
import ctre
from components.chassis import Chassis
from auto.trajectory import Trajectory, loadPath


class Robot(magicbot.MagicRobot):
    chassis: Chassis

    def createObjects(self):
        """Create motors and stuff here"""
        self.drive_motor_left = ctre.WPI_TalonSRX(1)
        self.drive_motor_right = ctre.WPI_TalonSRX(2)

        self.drive_motor_left.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.QuadEncoder, 0, 0
        )
        self.drive_motor_right.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.QuadEncoder, 0, 0
        )

        self.imu = ctre.PigeonIMU(3)

        self.driver = wpilib.joystick.Joystick(0)
        self.operator = wpilib.joystick.Joystick(1)

    def teleopInit(self):
        """Called when teleop starts; optional"""
        poses = loadPath("example.csv")
        trajectory = Trajectory(poses, 0.02)
        trajectory.build()

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""
        try:
            self.chassis.setInput(self.driver.getY(), self.driver.getZ())
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
