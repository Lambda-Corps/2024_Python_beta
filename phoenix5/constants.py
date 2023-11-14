from pyfrc.physics.units import units
import math

# Robot Physical Characteristics with dimensional units
ROBOT_MASS = 110 * units.lbs
DT_HIGH_GEAR_RATIO = 4.17
DT_LOW_GEAR_RATIO = 11.03
DT_GEAR_RATIO = DT_LOW_GEAR_RATIO # start with low gear as our sim
DT_MOTORS_PER_SIDE = 2
ROBOT_WHEELBASE = 22 * units.inch
ROBOT_BUMPER_WIDTH = 3.25 * units.inch
ROBOT_WIDTH = (23 * units.inch) + (ROBOT_BUMPER_WIDTH * 2)
ROBOT_LENGTH = (32 * units.inch) + (ROBOT_BUMPER_WIDTH * 2)
DT_WHEEL_DIAMETER = 6 * units.inch
DT_WHEEL_RADIUS_INCHES = DT_WHEEL_DIAMETER.m/2
DT_TRACKWIDTH_METERS = .546


# CAN IDS
DT_LEFT_LEADER      = 1 #1
DT_RIGHT_LEADER     = 2 #2
DT_LEFT_FOLLOWER    = 3 #3
DT_RIGHT_FOLLOWER   = 4 #4
#5
#6
#7
#8
#9
#10
#11
#12
#13
#14