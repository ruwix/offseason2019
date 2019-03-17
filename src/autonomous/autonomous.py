from magicbot import AutonomousStateMachine, timed_state, state
import wpilib

# this is one of your components
from components.chassis import Chassis
from components.autoselector import AutoSelector, AutoSide, AutoMode

from autonomous.ramsete import Ramsete
from autonomous.trajectory import Trajectory
from autonomous.paths import Path
import numpy as np


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Autonomous"
    DEFAULT = True

    # Injected from the definition in robot.py
    chassis: Chassis
    ramsete: Ramsete
    autoselector: AutoSelector

    def __init__(self):
        self.timer = wpilib.Timer()
        self.trajectory = None

    @state(first=True)
    def initMode(self, initial_call):
        side, mode = AutoSide.LEFT, AutoMode.ROCKET  # self.autoselector.getSelection()
        if mode == 0:
            self.next_state("crossLine")
        elif side == AutoSide.LEFT and mode == AutoMode.ROCKET:
            self.next_state("leftStartToRocket")
        else:
            self.next_state("stop")

    def followTrajectory(self):
        if self.trajectory == None:
            return False
        self.trajectory.update(self.timer.get())
        if not self.trajectory.isFinished():
            state = self.chassis.state
            state_d = self.trajectory.getState()
            twist = self.ramsete.update(state, state_d)
            print(self.ramsete.getError())
            wheels = self.chassis.getWheelVelocities(twist.v, twist.omega)
            self.chassis.setVelocity(wheels[0], wheels[1])
            return True
        else:
            return False

    @timed_state(duration=2, next_state="stop")
    def crossLine(self, initial_call):
        self.chassis.setVelocity(36, 36)

    @state
    def leftStartToRocket(self, initial_call):
        if initial_call:
            self.trajectory = Trajectory(
                Path.START_2_LEFT_ROCKET.getPoses(), 5, reversed=False
            )
            self.trajectory.build()
            self.trajectory.writeCSV("output.csv")
            self.trajectory.drawSimulation()
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("stop")

    @state
    def stop(self):
        self.chassis.setOutput(0, 0)
