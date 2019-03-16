from magicbot import AutonomousStateMachine, timed_state, state
import wpilib

# this is one of your components
from components.chassis import Chassis
from auto.ramsete import Ramsete
from auto.trajectory import Trajectory
from auto.paths import Path


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Autonomous"
    DEFAULT = True

    # Injected from the definition in robot.py
    chassis: Chassis
    ramsete: Ramsete

    def __init__(self):
        self.timer = wpilib.Timer()
        self.timestamp = 0
        self.last_timestamp = 0
        self.start_to_rocket = Trajectory(
            Path.START_2_LEFT_ROCKET.getPoses(), 4, inverted=False
        )

    @state(first=True)
    def startToRocket(self, initial_call):
        if initial_call:
            self.timer.start()
            self.start_to_rocket.build()
            self.start_to_rocket.writeCSV("output.csv")
            self.start_to_rocket.drawSimulation()

        self.timestamp = self.timer.get()
        self.start_to_rocket.update(self.timestamp)

        if not self.start_to_rocket.isFinished():
            state = self.chassis.state
            state_d = self.start_to_rocket.getState()
            twist = self.ramsete.update(state, state_d)
            print(self.ramsete.getError())
            wheels = self.chassis.getWheelVelocities(twist.v, twist.omega)
            self.chassis.setVelocity(wheels[0], wheels[1])
        else:
            self.next_state("stop")

    @state
    def stop(self):
        self.chassis.setOutput(0, 0)
