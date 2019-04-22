import wpilib

from components.chassis import Chassis
from components.localization import Localization
from controllers.ramsete import Ramsete
from trajectory.timedtrajectory import TimedTrajectory
from utils.physicalstates import ChassisState


class TrajectoryTracker:
    chassis: Chassis
    localization: Localization
    ramsete: Ramsete

    def __init__(self):
        self.timer = wpilib.Timer()
        self.previous_t = 0
        self.previous_velocity = None
        self.trajectory = None
        self.stopped = True

    def reset(self, trajectory: TimedTrajectory):
        self.trajectory = trajectory
        self.timer.reset()
        self.timer.start()
        self.previous_t = 0
        self.previous_velocity = None
        self.stopped = False

    def stop(self):
        self.stopped = True

    def isFinished(self):
        if self.trajectory == None:
            return True
        return self.timer.get() >= self.trajectory.length

    def update(self):
        if self.trajectory == None:
            raise ValueError("trajectory cannot be none, call reset to set it")

        self.stopped = False

        new_t = self.timer.get()
        dt = new_t - self.previous_t
        self.previous_t = new_t

        state = self.localization.state
        state_d = self.trajectory.getState(new_t)

        velocity = self.ramsete.update(state, state_d)
        if self.previous_velocity == None:
            acceleration = ChassisState(0, 0)
            return TrajectoryTrackerOutput(velocity, acceleration)
        else:
            linear_acceleration = (velocity.linear - self.previous_velocity.linear) / dt
            angular_acceleration = (
                velocity.angular - self.previous_velocity.angular
            ) / dt
            acceleration = ChassisState(linear_acceleration, angular_acceleration)
            return TrajectoryTrackerOutput(velocity, acceleration)

    def execute(self):
        if self.isFinished():
            self.stop()
        if not self.stopped:
            output = self.update()
            self.chassis.setVelocityFromTrajectoryTracker(output)


class TrajectoryTrackerOutput:
    def __init__(self, velocity: ChassisState, acceleration: ChassisState):
        self.velocity = velocity
        self.acceleration = acceleration
