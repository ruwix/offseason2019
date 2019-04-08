from utils.epsilon import epsilonEquals
import numpy as np
from utils.geometry import PoseWithCurvature
from utils.mathextension import lerp


class TimedState:
    EPSILON = 1e-6

    def __init__(
        self, state: PoseWithCurvature, t: float, velocity: float, acceleration: float
    ):
        self.state = state
        self.t = t
        self.velocity = velocity
        self.acceleration = acceleration

    def interpolate(self, other, t: float):
        if t <= self.EPSILON:
            return self
        elif abs(t + self.EPSILON) >= 1:
            return other
        new_t = lerp(self.t, other.t, t)
        delta_t = new_t - self.t
        if delta_t < 0:
            return other.interpolate(self, 1 - t)

        reversing = self.velocity < 0.0 or (
            epsilonEquals(self.velocity, 0) and self.acceleration < 0.0
        )
        new_v = self.velocity + self.acceleration * delta_t
        direction = 1 if not reversing else 1
        new_s = direction * (
            self.velocity * delta_t + 0.5 * self.acceleration * delta_t * delta_t
        )
        return TimedState(
            self.state.interpolate(
                other.state, new_s / self.state.distance(other.state)
            ),
            new_t,
            new_v,
            self.acceleration,
        )

    def __str__(self):
        return f"({self.state}, {self.t}, {self.velocity}, {self.acceleration}"


class TimedTrajectory:
    def __init__(self, timed_states: float):
        self.timed_states = timed_states
        self.length = self.timed_states[-1].t
        self.times = np.empty(len(timed_states))
        for i in range(len(self.timed_states)):
            self.times[i] = self.timed_states[i].t

    def getState(self, t: float) -> TimedState:
        if t <= 0:
            return self.timed_states[0]
        elif t >= self.length:
            return self.timed_states[-1]
        else:
            index = np.argmax(self.times > t)
            if epsilonEquals(self.times[index], self.times[index - 1]):
                return self.timed_states[index]
            else:
                return self.timed_states[index - 1].interpolate(
                    self.timed_states[index],
                    (t - self.times[index - 1])
                    / (self.times[index] - self.times[index - 1]),
                )
