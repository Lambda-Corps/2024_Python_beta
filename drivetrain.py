import wpilib.drive
from commands2 import Subsystem, Command
import phoenix6
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, FeedbackSensorSourceValue, 

import constants
import math
from pyfrc.physics.units import units

class DriveTrain(Subsystem):
    DT_TICKS_PER_MOTOR_REV = int(2048)
    DT_TICKS_PER_INCH = (DT_TICKS_PER_MOTOR_REV * constants.DT_HIGH_GEAR_RATIO) / ((2 * math.pi) * constants.DT_WHEEL_DIAMETER.m_as(units.inch))

    def __init__(self) -> None:
        super().__init__()

        self._left_leader = phoenix6.TalonFX(constants.DT_LEFT_LEADER)
        self._left_follower = phoenix6.TalonFX(constants.DT_LEFT_FOLLOWER)
        self._right_leader = phoenix6.TalonFX(constants.DT_RIGHT_LEADER)
        self._right_follower = phoenix6.TalonFX(constants.DT_LEFT_FOLLOWER)

        self.__configure_left_side_drive()
        self.__configure_right_side_drive()

    
    def __configure_left_side_drive(self) -> None:
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = phoenix6.TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
        
        # Apply the configuration to the motors
        self._left_leader.configurator.apply(config)
        self._left_follower.configurator.apply(config)

        # Set the left follower to follow the leader
        self._left_follower.set_control(Follower(constants.DT_LEFT_LEADER, False))

        self._left_leader.sim_state.Orientation = ChassisReference.COUNTER_CLOCKWISE_POSITIVE
        self._left_follower.sim_state.Orientation = ChassisReference.COUNTER_CLOCKWISE_POSITIVE


    def __configure_right_side_drive(self) -> None:
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = phoenix6.TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
        
        # Apply the configuration to the motors
        self._left_leader.configurator.apply(config)
        self._left_follower.configurator.apply(config)

        # Set the left follower to follow the leader
        self._left_follower.set_control(Follower(constants.DT_RIGHT_LEADER, False))

        self._left_leader.sim_state.Orientation = ChassisReference.CLOCKWISE_POSITIVE
        self._left_follower.sim_state.Orientation = ChassisReference.CLOCKWISE_POSITIVE