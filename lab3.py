"""lab3 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar
import math


MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.5725 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10 # Left wheel index
MOTOR_RIGHT = 11 # Right wheel index
N_PARTS = 12 # Total joints

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
# Note that positions for the joints are in radians.
target_pos = (0.0, 0.0, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts = []
#x: 2.344960;
#y: 1.112664;
#theta:
for i in range(N_PARTS):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))
        robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

##### Note that the following sensors will not be used for Lab 3.######

# We want to just show that these rich sensors are available with Tiago
# some of which will be used later.Feel free to play with them.
# The Tiago robot has a couple more sensors than the e-Puck.
# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
#camera.enable(timestep)
#camera.recognitionEnable(timestep)
# lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
#lidar.enable(timestep)
#######################################################################

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

waypoints = [(1.9, -2.5), (1.37, -2.43), (1.44, -3.0)]
current_waypoint = (2.0, 0.0)

while robot.step(timestep) != -1:

    ## webots offer builtin object detectors which will not be used in Lab 3 ###

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Webots simulates the equivalent of a Mask R-CNN deep learning-based object
    # recognition framework. We print out the names of the objects Tiago sees. You
    # can plan with this data structure eventually.
    #objects = camera.getRecognitionObjects()
    #for i in objects:
    #    print(i.get_model())
    ##########################################################################

    dist_err = math.sqrt(math.pow(pose_x - current_waypoint[0], 2) + math.pow(pose_y - current_waypoint[1], 2))
    bearing_err = pose_theta - math.atan2(current_waypoint[1] - pose_y, current_waypoint[0] - pose_x)

    #STEP 2: Controller (with gains)

    x_gain = 1.5
    theta_gain = 3.1 
    x_prime = dist_err * x_gain
    theta_prime = (theta_gain * bearing_err)
    
    if(len(waypoints) != 0 and dist_err < 0.1):
        current_waypoint = waypoints.pop(0)

    print("Distance Error: %f, Bearing Error: %f x prime: %f" % (dist_err, bearing_err, x_prime))
    print("Target x: %f Target y: %f" % (current_waypoint[0], current_waypoint[1]))

    #STEP 3: Compute wheelspeeds

    
    if(bearing_err > 0.01):
        vR = theta_prime + theta_gain
        vL = -theta_prime - theta_gain
    elif(bearing_err < -0.01):
        vR = -theta_prime - theta_gain
        vL = theta_prime + theta_gain
    else: 
        vR = x_prime + x_gain
        vL = x_prime + x_gain

    #STEP 4: Normalize wheelspeed
    
    if (vR > MAX_SPEED):
        vR = MAX_SPEED
    elif(vR < -MAX_SPEED):
        vR = -MAX_SPEED
    if (vL > MAX_SPEED):
        VL = MAX_SPEED
    elif(vL < -MAX_SPEED):
        vL = -MAX_SPEED
        
    #print("Left Velocity: %f, Right Velocity: %f", (vL, vR))

    ################# Do not modify this block of the code ########################
    # Odometry code. Don't change speeds after this
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vL-vR)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
    print("X: %f Y: %f Theta: %f" % (pose_x, pose_y, pose_theta)) # Comment out this if you want
    ##############################################################################

    # Enter here functions to send actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
   
    
