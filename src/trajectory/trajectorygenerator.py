import numpy as np
from trajectory.quintichermitespline import QuinticHermiteSpline

from trajectory.cubichermitespline import CubicHermiteSpline

from pyfrc.sim import get_user_renderer

from utils.geometry import RobotState, boundRadians, Vector, Pose, PoseWithCurvature
from utils import units
from wpilib import RobotBase
from trajectory.splinegenerator import parameterizeSplines
from trajectory.distancetrajectory import DistanceTrajectory
from trajectory.timedtrajectory import TimedState, TimedTrajectory


class TrajectoryContraints:
    def __init__(self, max_velocity: float):
        self.max_velocity = max_velocity


class TrajectoryGenerator:
    def __init__(
        self, max_dx=0.05, max_dy=0.00635, max_dtheta=5 * units.degrees_per_radian
    ):
        self.max_dx = max_dx
        self.max_dy = max_dy
        self.max_dtheta = max_dtheta

    def generateTrajectory(
        self,
        spline_poses: np.array,
        constrains: TrajectoryContraints,
        start_velocity,
        end_velocity,
        max_velocity,
        max_acceleration,
        _reversed=False,
    ):
        # Make theta normal for trajectory generation if path is trajectoryReversed.

        if _reversed:
            for i in range(0, len(spline_poses)):
                spline_poses[i].theta += np.pi
        splines = np.empty(len(spline_poses) - 1, dtype=QuinticHermiteSpline)
        for i in range(0, len(spline_poses) - 1):
            splines[i] = QuinticHermiteSpline(spline_poses[i], spline_poses[i + 1])
        d = self.getDistanceTrajectory(splines)
        t = self.timeParameterizeTrajectory(
            d,
            constrains,
            start_velocity,
            end_velocity,
            max_velocity,
            max_acceleration,
            self.max_dx,
            _reversed,
        )
        return t

    def getTrajectoryPosesFromSplines(self, splines):
        return parameterizeSplines(splines, self.max_dx, self.max_dy, self.max_dtheta)

    def getDistanceTrajectory(self, splines):
        t = self.getTrajectoryPosesFromSplines(splines)
        return DistanceTrajectory(t)

    def timeParameterizeTrajectory(
        self,
        d_trajectory,
        constrains: TrajectoryContraints,
        start_velocity,
        end_velocity,
        max_velocity,
        max_acceleration,
        step_size,
        _reversed,
    ):
        class ConstrainedState:
            def __init__(
                self,
                state: PoseWithCurvature = PoseWithCurvature(),
                distance: float = 0,
                max_velocity: float = 0,
                min_acceleration: float = 0,
                max_acceleration: float = 0,
            ):
                self.state = state
                self.distance = distance
                self.max_velocity = max_velocity
                self.min_acceleration = min_acceleration
                self.max_acceleration = max_acceleration

            def __str__(self):
                return f"({self.state}, {self.distance}, {self.max_velocity}, {self.min_acceleration}, {self.max_acceleration}"

        states = np.empty(0, dtype=PoseWithCurvature)
        t = 0
        while t < d_trajectory.length:
            pose = d_trajectory.getPoseWithCurvature(t)
            states = np.append(states, pose)
            t += step_size
        constrained_states = np.empty(len(states), dtype=ConstrainedState)
        epsilon = 1e-6

        # Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
        # parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
        # acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
        # there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
        # velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).

        predecessor = ConstrainedState(
            states[0], 0, start_velocity, -max_acceleration, max_acceleration
        )
        for i in range(len(states)):
            # Add the new state.

            constrained_states[i] = ConstrainedState()
            constrained_state = constrained_states[i]
            constrained_state.state = states[i]
            ds = constrained_state.state.getDistance(predecessor.state)
            constrained_state.distance = ds + predecessor.distance
            # We may need to iterate to find the maximum end velocity and common acceleration, since acceleration
            # limits may be a function of velocity.

            while True:
                # Enforce global max velocity and max reachable velocity by global acceleration limit.
                # vf = sqrt(vi^2 + 2*a*d)
                constrained_state.max_velocity = np.min(
                    (
                        max_velocity,
                        np.sqrt(
                            predecessor.max_velocity ** 2
                            + 2 * predecessor.max_acceleration * ds
                        ),
                    )
                )
                if np.isnan(constrained_state.max_velocity):
                    raise ValueError("Max Velocity cannot be NaN")
                # Enforce global max absolute acceleration.

                constrained_state.min_acceleration = -max_acceleration
                constrained_state.max_acceleration = max_acceleration

                # At this point, the state is full constructed, but no constraints have been applied aside from
                # predecessor
                # state max accel.

                # Enforce all velocity constraints.
                # TODO apply constraint constraints
                if constrained_state.max_velocity < 0:
                    # This should never happen if constraints are well-behaved.
                    raise ValueError("Max Velocity cannot be negative")
                # Now enforce all acceleration constraints.

                # TODO apply acceleration constraints
                if (
                    constrained_state.min_acceleration
                    > constrained_state.max_acceleration
                ):
                    # This should never happen if constraints are well-behaved.
                    raise ValueError(
                        "Min Acceleration cannot be greater tha Max Acceleration"
                    )

                if ds > epsilon:
                    break

                # If the max acceleration for this constraint state is more conservative than what we had applied, we
                # need to reduce the max accel at the predecessor state and try again.
                actual_acceleration = (
                    constrained_state.max_velocity ** 2 - predecessor.max_velocity ** 2
                ) / (2.0 * ds)
                if constrained_state.max_acceleration < actual_acceleration - epsilon:
                    predecessor.max_acceleration = constrained_state.max_acceleration
                else:
                    # If actual acceleration is less than predecessor min accel, we will repair during the backward
                    # pass.
                    if actual_acceleration > predecessor.min_acceleration + epsilon:
                        predecessor.max_acceleration = actual_acceleration
                    break

        # Backward pass.
        successor = ConstrainedState()
        successor.state = states[-1]
        successor.distance = constrained_states[-1].distance
        successor.max_velocity = end_velocity
        successor.min_acceleration = -max_acceleration
        successor.max_acceleration = max_acceleration
        for i in reversed(range(len(states))):
            constrained_state = constrained_states[i]
            ds = constrained_state.distance - successor.distance
            while True:
                # Enforce reverse max reachable velocity limit.
                # vf = sqrt(vi^2 + 2*a*d), where vi = successor.
                new_max_velocity = np.sqrt(
                    successor.max_velocity ** 2 + 2.0 * successor.min_acceleration * ds
                )
                if new_max_velocity >= constrained_state.max_velocity:
                    # No new limits to impose.
                    break
                constrained_state.max_velocity = new_max_velocity
                if np.isnan(constrained_state.max_velocity):
                    raise ValueError("Max Velocity cannot be NaN")

                # Now check all acceleration constraints with the lower max velocity.
                # TODO check acceleration constraints
                if (
                    constrained_state.min_acceleration
                    > constrained_state.max_acceleration
                ):
                    raise ValueError(
                        "Min Acceleration cannot be greater than Max Acceleration"
                    )

                if ds > epsilon:
                    break

                # If the min acceleration for this constraint state is more conservative than what we have applied, we
                # need to reduce the min accel and try again.
                actual_acceleration = (
                    constrained_state.max_velocity ** 2 - successor.max_velocity ** 2
                ) / (2.0 * ds)
                if constrained_state.min_acceleration < actual_acceleration - epsilon:
                    successor.min_acceleration = constrained_state.min_acceleration
                else:
                    successor.min_acceleration = actual_acceleration
                    break
            successor = constrained_state

        # Integrate the constrained states forward in time to obtain the TimedStates.
        timed_states = np.empty(len(states), dtype=TimedState)
        t = 0
        s = 0
        v = 0
        for i in range(len(states)):
            constrained_state = constrained_states[i]
            # Advance t.
            ds = constrained_state.distance - s
            numerator = constrained_state.max_velocity ** 2 - v ** 2
            if numerator == 0:
                accel = 0
            else:
                accel = numerator / (2 * ds)
            dt = 0.0
            if i > 0:
                if _reversed:
                    timed_states[i - 1].acceleration *= -1
                if abs(accel) > epsilon:
                    dt = (constrained_state.max_velocity - v) / accel
                elif abs(v) > epsilon:
                    dt = ds / v
                else:
                    raise RuntimeError("No valid dt found")
            t += dt
            if np.isnan(t) or np.isinf(t):
                raise RuntimeError("t cannot be NaN or infinite")

            v = constrained_state.max_velocity
            s = constrained_state.distance
            timed_states[i] = TimedState(
                constrained_state.state,
                t,
                v if not _reversed else -v,
                accel if not _reversed else -accel,
            )
        return TimedTrajectory(timed_states)

    def drawSimulation(self, timed_trajectory) -> None:
        if RobotBase.isSimulation() and get_user_renderer() != None:
            points = np.empty((0, 2))
            for timed_state in timed_trajectory.timed_states:
                state = timed_state.state
                point = np.array([state.pose.x, state.pose.y + 4.1148]).reshape((1, 2))
                points = np.append(points, point, axis=0)
            get_user_renderer().draw_line(
                points,
                scale=(units.feet_per_meter, units.feet_per_meter),
                color="#FFFFFF",
                width=2,
            )

