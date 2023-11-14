import wpilib
import wpilib.drive
from commands2 import SubsystemBase
from phoenix5 import WPI_TalonFX, TalonFXInvertType, NeutralMode, FeedbackDevice
import constants
import math
from pyfrc.physics.units import units
from typing import List

class DriveTrain(SubsystemBase):
    DT_TICKS_PER_MOTOR_REV = int(2048) # Falcons are 2048
    DT_TICKS_PER_INCH = (DT_TICKS_PER_MOTOR_REV * constants.DT_GEAR_RATIO) / ((2 * math.pi ) * constants.DT_WHEEL_DIAMETER.m_as(units.inch))

    def __init__(self) -> None:
        super().__init__()

        self._left_leader = WPI_TalonFX(constants.DT_LEFT_LEADER)
        self._left_follower = WPI_TalonFX(constants.DT_LEFT_FOLLOWER)
        self._right_leader = WPI_TalonFX(constants.DT_RIGHT_LEADER)
        self._right_follower = WPI_TalonFX(constants.DT_RIGHT_FOLLOWER)

        # Factory default the motor controllers
        self._left_leader.configFactoryDefault()
        self._left_follower.configFactoryDefault()
        self._right_leader.configFactoryDefault()
        self._right_follower.configFactoryDefault()

        # # Motor are mounted opposite of each other, so one needs to run "backward" to make the robot move
        # in the direction we want.
        self._right_leader.setInverted(TalonFXInvertType.CounterClockwise)
        self._left_leader.setInverted(TalonFXInvertType.Clockwise)

        # Set the follower motors, and their inversions to match
        self._left_follower.follow(self._left_leader)
        self._left_follower.setInverted(TalonFXInvertType.FollowMaster)
        self._right_follower.follow(self._right_leader)
        self._right_follower.setInverted(TalonFXInvertType.FollowMaster)

        # Set the neutral mode to brake so the robot stops more responsively
        self._left_leader.setNeutralMode(NeutralMode.Brake)
        self._right_leader.setNeutralMode(NeutralMode.Brake)

        # Set the integrated encoder as the primary feedback sensor and force it's starting position to 0
        self._left_leader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0)
        self._right_leader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0)
        self._left_leader.setSelectedSensorPosition(0)
        self._right_leader.setSelectedSensorPosition(0)

        if wpilib.RobotBase.isSimulation():
            self._left_motor_sim = self._left_leader.getSimCollection()
            self._right_motor_sim = self._right_leader.getSimCollection()


    def driveManually(self, forward: float, turn: float):
        '''Use the member variable _diffdrive to set the left and right side motors based on the inputs. The third argument being passed in true represents whether or not we should allow the robot to turn in place or not'''
        wheelSpeeds = wpilib.drive.DifferentialDrive.arcadeDriveIK(forward, turn, True)
        
        self._left_leader.set(wheelSpeeds.left)
        self._right_leader.set(wheelSpeeds.right)

        self._lastLeftSet = wheelSpeeds.left
        self._lastRightSet = wheelSpeeds.right