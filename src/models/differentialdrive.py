"""
Implementation from:
FRC Team 5190
Green Hope Falcons;
Who implemented from:
NASA Ames Robotics "The Cheesy Poofs"
Team 254
"""
import numpy as np

from models.dcmotortransmission import DCMotorTransmission
from utils.epsilon import EPSILON, epsilonEquals
from utils.mathextension import MinMax
from utils.physicalstates import ChassisState, DriveDynamics, WheelState


class DifferentialDrive:
    """Dynamic model a differential drive robot. Note: to simplify things, this math assumes the center of mass is
       coincident with the kinematic center of rotation (e.g. midpoint of the center axle)."""

    def __init__(
        self,
        mass: float,
        moi: float,
        angular_drag: float,
        wheel_radius: float,
        effective_wheel_base_radius: float,
        left_transmission: DCMotorTransmission,
        right_transmission: DCMotorTransmission,
    ):
        """
        Equivalent mass when accelerating purely linearly, in kg.
        This is equivalent in that it also absorbs the effects of drivetrain inertia.
        Measure by doing drivetrain acceleration characterizaton in a straight line.
        """
        self.mass = mass
        """
        Equivalent moment of inertia when accelerating purely angularly in kg m^2.
        This is equivalent in that it also absorbs the effects of drivetrain inertia.
        Measure by doing drivetrain acceleration characterization while turing in place.
        """
        self.moi = moi

        """
        Drag torque (proportional to angular velocity) that resists turning in Nm/rad/s.
        Empirical testing of 254's drivebase showed that there was an unexplained loss in torque ~proportional to angular
        velocity, likely due to scrub of wheels.
        NOTE: this may not be a purely linear tern, and 254 has done limited testing, but this factor may help a model
        to better match reality.
        """
        self.angular_drag = angular_drag

        """The radius of the wheel."""
        self.wheel_radius = wheel_radius

        """
        Effective kinematic wheelbase radius. Might be larger than theoretical to compensate for skid steer. Measure by
        turning the robot in place several times and figuring out what the equivalent wheel base radius is.
        """
        self.effective_wheel_base_radius = effective_wheel_base_radius

        """ DC Motor transmission for both sides of the drivetrain."""
        self.left_transmission = left_transmission
        self.right_transmission = right_transmission

    def solveForwardKinematics(self, wheel_motion: WheelState) -> ChassisState:
        """
        Solve forward kinematics to get chassis motion from wheel motion.
        Could be either acceleration or velocity.
        """
        linear = self.wheel_radius * (wheel_motion.right + wheel_motion.left) / 2.0
        angular = (
            self.wheel_radius
            * (wheel_motion.right - wheel_motion.left)
            / (2.0 * self.effective_wheel_base_radius)
        )
        return ChassisState(linear, angular)

    def solveInverseKinematics(self, chassis_motion: ChassisState) -> WheelState:
        """
        Solve inverse kinematics to get wheel motion from chassis motion.
        Could be either acceleration or velocity.
        """
        left = (
            chassis_motion.linear
            - self.effective_wheel_base_radius * chassis_motion.angular
        ) / self.wheel_radius
        right = (
            chassis_motion.linear
            + self.effective_wheel_base_radius * chassis_motion.angular
        ) / self.wheel_radius
        return WheelState(left, right)

    def getVoltagesFromkV(self, velocities: WheelState) -> WheelState:
        """Get the voltage simply from the Kv and the friction voltage of the transmissions."""
        return WheelState(
            velocities.left / self.left_transmission.speedPerVolt
            + self.left_transmission.frictionVoltage * np.sign(velocities.left),
            velocities.right / self.right_transmission.speedPerVolt
            + self.right_transmission.frictionVoltage * np.sign(velocities.right),
        )

    def solveForwardWheelDynamics(
        self, wheel_velocity: WheelState, voltage: WheelState
    ) -> DriveDynamics:
        """Solve forward dynamics for torques and accelerations."""
        chassis_velocity = self.solveForwardKinematics(wheel_velocity)
        return self.solveForwardDynamics(
            wheel_velocity,
            chassis_velocity,
            (chassis_velocity.angular / chassis_velocity.linear),
            voltage,
        )

    def solveForwardChassisDynamics(
        self, chassis_velocity: ChassisState, voltage: WheelState
    ) -> DriveDynamics:
        """Solve forward dynamics for torques and accelerations."""
        return self.solveForwardDynamics(
            solveInverseKinematics(chassis_velocity),
            chassis_velocity,
            (chassis_velocity.angular / chassis_velocity.linear),
            voltage,
        )

    def solveForwardDynamics(
        self,
        wheel_velocity: WheelState,
        chassis_velocity: ChassisState,
        curvature: float,
        voltage: float,
    ) -> DriveDynamics:
        """Solve forward dynamics for torques and accelerations."""
        left_stationary = (
            epsilonEquals(wheel_velocity.left, 0)
            and abs(voltage.left) < self.left_transmission.friction_voltage
        )
        right_stationary = (
            epsilonEquals(wheel_velocity.right, 0)
            and abs(voltage.right) < self.right_transmission.friction_voltage
        )

        # Neither side breaks static friction, so we remain stationary.
        if left_stationary and right_stationary:
            wheel_torque = WheelState(0, 0)
            chassis_acceleration = ChassisState(0, 0)
            wheel_acceleration = WheelState(0, 0)
            dcurvature = 0
        else:
            # Solve for motor torques generated on each side.
            wheel_torque = WheelState(
                self.left_transmission.getTorqueFromVoltage(
                    wheel_velocity.left, voltage.left
                ),
                self.right_transmission.getTorqueFromVoltage(
                    wheel_velocity.right, voltage.right
                ),
            )

            # Add forces and torques about the center of mass.
            chassis_acceleration = ChassisState(
                (wheel_torque.right + wheel_torque.left)
                / (self.wheel_radius * self.mass),
                self.effective_wheel_base_radius
                * (wheel_torque.right - wheel_torque.left)
                / (self.wheel_radius * self.moi)
                - chassis_velocity.angular * self.angular_drag / self.moi,
            )

            # Solve for change in curvature from angular acceleration.
            # total angular accel = linear_accel * curvature + v^2 * dcurvature
            dcurvature = (
                chassis_acceleration.angular - chassis_acceleration.linear * curvature
            ) / (chassis_velocity.linear * chassis_velocity.linear)

            # Resolve chassis accelerations to each wheel.
            wheel_acceleration = WheelState(
                chassis_acceleration.linear
                - chassis_acceleration.angular * self.effective_wheel_base_radius,
                chassis_acceleration.linear
                + chassis_acceleration.angular * self.effective_wheel_base_radius,
            )
        return DriveDynamics(
            curvature,
            dcurvature,
            chassis_velocity,
            chassis_acceleration,
            wheel_velocity,
            wheel_acceleration,
            voltage,
            wheel_torque,
        )

    def solveInverseChassisDynamics(
        self, chassis_velocity: ChassisState, chassis_acceleration: ChassisState
    ) -> DriveDynamics:
        """Solve inverse dynamics for torques and voltages."""
        curvature = chassis_velocity.angular / chassis_velocity.linear
        dcurvature = (
            chassis_acceleration.angular - chassis_acceleration.linear * curvature
        ) / (chassis_velocity.linear ** 2)
        return self.solveInverseDynamics(
            solveInverseKinematics(chassis_velocity),
            chassis_velocity,
            solveInverseKinematics(chassis_acceleration),
            chassis_acceleration,
            curvature,
            dcurvature,
        )

    def solveInverseWheelDynamics(
        self, wheel_velocity: WheelState, wheel_acceleration: WheelState
    ) -> DriveDynamics:
        """Solve inverse dynamics for torques and voltages."""
        chassis_velocity = self.solveForwardKinematics(wheel_velocity)
        chassis_acceleration = self.solveForwardKinematics(wheel_acceleration)

        curvature = chassis_velocity.angular / chassis_acceleration.linear

        dcurvature = (
            chassis_acceleration.angular - chassis_acceleration.linear * curvature
        ) / (chassis_velocity.linear * chassis_velocity.linear)

        return self.solveInverseDynamics(
            wheel_velocity,
            chassis_velocity,
            wheel_acceleration,
            chassis_acceleration,
            curvature,
            dcurvature,
        )

    def solveInverseDynamics(
        self,
        wheel_velocity: WheelState,
        chassis_velocity: ChassisState,
        wheel_acceleration: WheelState,
        chassis_acceleration: ChassisState,
        curvature: float,
        dcurvature: float,
    ) -> DriveDynamics:
        """Solve inverse dynamics for torques and voltages."""

        # Determine the necessary torques on the left and right wheels to produce the desired wheel accelerations.
        wheel_torque = WheelState(
            self.wheel_radius
            / 2.0
            * (
                chassis_acceleration.linear * self.mass
                - chassis_acceleration.angular
                * self.moi
                / self.effective_wheel_base_radius
                - chassis_velocity.angular
                * self.angular_drag
                / self.effective_wheel_base_radius
            ),
            self.wheel_radius
            / 2.0
            * (
                chassis_acceleration.linear * self.mass
                + chassis_acceleration.angular
                * self.moi
                / self.effective_wheel_base_radius
                + chassis_velocity.angular
                * self.angular_drag
                / self.effective_wheel_base_radius
            ),
        )
        # Solve for input voltages
        voltage = WheelState(
            self.left_transmission.getVoltageFromTorque(
                wheel_velocity.left, wheel_torque.left
            ),
            self.right_transmission.getVoltageFromTorque(
                wheel_velocity.right, wheel_torque.right
            ),
        )
        return DriveDynamics(
            curvature,
            dcurvature,
            chassis_velocity,
            chassis_acceleration,
            wheel_velocity,
            wheel_acceleration,
            voltage,
            wheel_torque,
        )

    def getMaxAbsVelocity(self, curvature: float, max_abs_voltage: float) -> float:
        """
        Curvature is redundant here in the case that chassisVelocity is not purely angular.
        It is the responsibility of the caller to ensure that curvature = angular vel / linear vel in these cases.
        """
        left_speed_at_max_moltage = self.left_transmission.getFreeSpeedAtVoltage(
            max_abs_voltage
        )
        right_speed_at_max_moltage = self.right_transmission.getFreeSpeedAtVoltage(
            max_abs_voltage
        )

        if epsilonEquals(curvature, 0):
            return self.wheel_radius * np.min(
                left_speed_at_max_moltage, right_speed_at_max_moltage
            )

        if np.isinf(curvature):
            # Turn in place. Return value meaning becomes angular velocity.
            wheel_speed = np.min(left_speed_at_max_moltage, right_speed_at_max_moltage)
            return (
                np.sign(curvature)
                * self.wheel_radius
                * wheel_speed
                / self.effective_wheel_base_radius
            )

        right_speed_if_left_max = (
            left_speed_at_max_moltage
            * (self.effective_wheel_base_radius * curvature + 1.0)
            / (1.0 - self.effective_wheel_base_radius * curvature)
        )

        if np.abs(right_speed_if_left_max) <= right_speed_at_max_moltage + EPSILON:
            # Left max is active constraint.
            return (
                self.wheel_radius
                * (left_speed_at_max_moltage + right_speed_if_left_max)
                / 2.0
            )
        left_speed_if_left_max = (
            right_speed_at_max_moltage
            * (1.0 - self.effective_wheel_base_radius * curvature)
            / (1.0 + self.effective_wheel_base_radius * curvature)
        )

        # Right at max is active constraint.
        return (
            self.wheel_radius
            * (right_speed_at_max_moltage + left_speed_if_left_max)
            / 2.0
        )

    def getMinMaxAcceleration(
        self, chassis_velocity: ChassisState, curvature: float, max_abs_voltage: float
    ):
        wheel_velocities = solveInverseKinematics()
        min = np.Inf
        max = np.NINF
        # Math:
        # (Tl + Tr) / r_w = m*a
        # (Tr - Tl) / r_w * r_wb - drag*w = i*(a * k + v^2 * dk)

        # 2 equations, 2 unknowns.
        # Solve for a and (Tl|Tr)
        if np.isinf(curvature):
            linear_term = 0.0
            angular_term = self.moi
        else:
            linear_term = mass * self.effective_wheel_base_radius
            angular_term = self.moi * curvature
        drag_torque = chassis_velocity.angular * angular_drag

        # Check all four cases and record the min and max valid accelerations.
        for left in (False, True):
            for sign in (1, -1):
                fixed_wheel_velocity = (
                    wheel_velocities.left if left else wheel_velocities.right
                )
                variable_wheel_velocity = (
                    wheel_velocities.right if left else wheel_velocities.left
                )
                fixed_transmission = (
                    self.left_transmission if left else self.right_transmission
                )
                variable_transmission = (
                    self.right_transmission if left else self.left_transmission
                )
                fixed_torque = fixedTransmission.getTorqueForVoltage(
                    fixed_wheel_velocity, sign * max_abs_voltage
                )

                # NOTE: variable_torque is wrong. Units don't work out correctly. We made a math error somewhere...
                # Leaving this "as is" for code release so as not to be disingenuous, but this whole function needs
                # revisiting in the future...
                if left:
                    variable_torque = (
                        -drag_torque * self.mass * self.wheel_radius
                        + fixed_torque * (linear_term + angular_term)
                    ) / (linear_term - angular_term)
                else:
                    variable_torque = (
                        drag_torque * self.mass * self.wheel_radius
                        + fixed_torque * (linear_term - angular_term)
                    ) / (linear_term + angular_term)
                variable_voltage = variable_transmission.getVoltageFromTorque(
                    variable_wheel_velocity, variable_torque
                )
                if np.abs(variable_voltage) <= max_abs_voltage + EPSILON:
                    if np.isinf(curvature):
                        sign = -1 if left else 1
                        accel = (
                            sign
                            * (fixed_torque - variable_torque)
                            * self.effective_wheel_base_radius
                            / (self.moi * self.wheel_radius)
                            - drag_torque / self.moi
                        )
                    else:
                        accel = (fixed_torque + variable_torque) / (
                            self.mass / self.moi
                        )
                    min = np.min(min, accel)
                    max = np.max(max, accel)
        return MinMax(min, max)
