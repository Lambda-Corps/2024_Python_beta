import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
from wpimath.system.plant import DCMotor, LinearSystemId
import math
import typing
import phoenix6

import constants

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    DT_TICKS_PER_MOTOR_REV = int(2048)  # Falcons are 2048
    DT_TICKS_PER_INCH = (DT_TICKS_PER_MOTOR_REV * constants.DT_GEAR_RATIO) / (
        (2 * math.pi) * constants.DT_WHEEL_DIAMETER.m_as(units.inch)
    )
    DT_TICKS_PER_METER = (DT_TICKS_PER_MOTOR_REV * constants.DT_GEAR_RATIO) / (
        (2 * math.pi) * constants.DT_WHEEL_DIAMETER.m_as(units.meter)
    )

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # Keep a reference to the robot passed in
        self._robot_instance = robot

        # Create a sim Gyro to be used for maintaining the heading
        # self._gyro = wpilib.simulation.AnalogGyroSim(robot._gyro)
        self._gyro = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self._gyro.getDouble("Yaw")

        self.position = 0

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch
        # fmt: off
        self._system = LinearSystemId.identifyDrivetrainSystem(1.98, .2, 1.5, .3)
        self._drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            self._system,
            constants.DT_TRACKWIDTH_METERS,
            DCMotor.falcon500(4),
            constants.DT_GEAR_RATIO,
            constants.DT_WHEEL_RADIUS_INCHES,
        )
        # fmt: on
        self._l_lead_motor: phoenix6.sim.TalonFXSimState = (
            robot._drivetrain._left_leader.sim_state
        )
        self._l_follow_motor: phoenix6.sim.TalonFXSimState = (
            robot._drivetrain._left_follower.sim_state
        )
        self._r_lead_motor: phoenix6.sim.TalonFXSimState = (
            robot._drivetrain._right_leader.sim_state
        )
        self._r_follow_motor: phoenix6.sim.TalonFXSimState = (
            robot._drivetrain._right_follower.sim_state
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.
        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # CTRE simulation is low-level, it ignores some of the things like motor
        # motor invresion etc.  WPILib wants +v to be forward.
        # Start the motor simulation work flow by passing robot battery voltage to sim motors
        self._l_lead_motor.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        self._r_lead_motor.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        self._l_follow_motor.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        self._r_follow_motor.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )

        # Apply the motor inputs to the simulation
        self._drivesim.setInputs(
            self._l_lead_motor.motor_voltage,
            self._r_lead_motor.motor_voltage,
        )

        # advance the simulation model a timing loop
        self._drivesim.update(tm_diff)

        # transform = self.drivetrain.calculate(speeds[self.LEFT_SPEED_INDEX], speeds[self.RIGHT_SPEED_INDEX], tm_diff)
        # pose = self.physics_controller.move_robot(transform)
        self._l_lead_motor.set_raw_rotor_position(
            self.__feet_to_encoder_ticks(self._drivesim.getLeftPositionFeet())
        )
        self._l_lead_motor.set_rotor_velocity(
            self.__velocity_feet_to_talon_ticks(self._drivesim.getLeftVelocityFps())
        )
        self._r_lead_motor.set_raw_rotor_position(
            self.__feet_to_encoder_ticks(self._drivesim.getRightPositionFeet())
        )
        self._r_lead_motor.set_rotor_velocity(
            self.__velocity_feet_to_talon_ticks(self._drivesim.getRightVelocityFps())
        )
        self._l_follow_motor.set_raw_rotor_position(
            self.__feet_to_encoder_ticks(self._drivesim.getLeftPositionFeet())
        )
        self._l_follow_motor.set_rotor_velocity(
            self.__velocity_feet_to_talon_ticks(self._drivesim.getLeftVelocityFps())
        )
        self._r_follow_motor.set_raw_rotor_position(
            self.__feet_to_encoder_ticks(self._drivesim.getRightPositionFeet())
        )
        self._r_follow_motor.set_rotor_velocity(
            self.__velocity_feet_to_talon_ticks(self._drivesim.getRightVelocityFps())
        )

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        pose = self._drivesim.getPose()
        self.navx_yaw.set(-self._drivesim.getHeading().degrees())
        # self.navx_yaw.set(-pose.rotation().degrees())

        self.physics_controller.field.setRobotPose(pose)

    def __feet_to_encoder_ticks(self, distance_in_feet: float) -> int:
        return int((distance_in_feet * 12) * self.DT_TICKS_PER_INCH)

    def __velocity_feet_to_talon_ticks(self, velocity_in_feet: float) -> int:
        wheel_rotations_per_second = (velocity_in_feet * 12) / (
            2 * math.pi * constants.DT_WHEEL_DIAMETER.m_as(units.inch)
        )
        wheel_rotations_per_100ms = (
            wheel_rotations_per_second * constants.DT_GEAR_RATIO
        ) / 10
        motor_rotations_per_100ms = wheel_rotations_per_100ms * constants.DT_GEAR_RATIO
        return int(motor_rotations_per_100ms * self.DT_TICKS_PER_MOTOR_REV)

    def __meters_to_encoder_ticks(self, distance_in_meters: float) -> int:
        return int((distance_in_meters * self.DT_TICKS_PER_METER))

    def __velocity_meters_to_talon_ticks(self, velocity_in_meters: float) -> int:
        wheel_rotations_per_second = velocity_in_meters / (
            2 * math.pi * constants.DT_WHEEL_DIAMETER.m_as(units.meter)
        )
        wheel_rotations_per_100ms = (
            wheel_rotations_per_second * constants.DT_GEAR_RATIO
        ) / 10
        motor_rotations_per_100ms = wheel_rotations_per_100ms * constants.DT_GEAR_RATIO
        return int(motor_rotations_per_100ms * self.DT_TICKS_PER_MOTOR_REV)
