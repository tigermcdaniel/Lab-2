"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from tkinter.tix import MAX

from cv2 import FarnebackOpticalFlow
from controller import Robot, Motor, DistanceSensor
import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
# TODO: Fill this in with a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 500

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 2
CENTER_IDX = 1
RIGHT_IDX = 0

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1259 # TODO*: To be filled in with ePuck wheel speed in m/s
MAX_SPEED = 6.28 # radians/sec = 0.25 m/s
#Calculated this by allowing the robot to drive forward at max speed for 3 seconds. Then, recorded the translation and divided by 3 to get 0.1259 m/s.

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 2.0 # TODO: Initialize variable for left speed
vR = 2.0 # TODO: Initialize variable for right speed
#radians/second 

#flag
encountered_line = False
turningLeft = False
turningRight = False
straight = 0

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    print(gsr) # TODO: Uncomment to see the ground sensor values!

    # Hints: 
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #
    
    # TODO: Insert Line Following Code Here  : state: follow_line      

    if turningLeft: 
        print("turning left" + str(straight))
        if straight > 2: 
            straight = 0
            vL = -1*MAX_SPEED
            vR = MAX_SPEED
        else: 
            straight += 1
            vL = MAX_SPEED
            vR = MAX_SPEED
    elif turningRight: 
        print("turning right" + str(turningRight))
        if straight > 2: 
            straight = 0
            vL = MAX_SPEED
            vR = -1*MAX_SPEED
        else: 
            straight += 1
            vL = MAX_SPEED
            vR = MAX_SPEED
    else: 
        # a) If the center ground sensor detects the line, the robot should drive forward.
        if gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[0] > GROUND_SENSOR_THRESHOLD and gsr[2] > GROUND_SENSOR_THRESHOLD:
            vL = MAX_SPEED
            vR = MAX_SPEED
            turningLeft = False
            turningRight = False
        # b) If the left ground sensor detects the line, the robot should rotate counterclockwise in place.
        elif gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[2] > GROUND_SENSOR_THRESHOLD: 
            vL = -1*MAX_SPEED
            vR = MAX_SPEED
            turningLeft = True
            turningRight = False

        # c) If the right ground sensor detects the line, the robot should rotate clockwise in place.
        elif gsr[2] < GROUND_SENSOR_THRESHOLD and gsr[0] > GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD: 
            vL = MAX_SPEED
            vR = -1*MAX_SPEED
            turningRight = True
            turningLeft= False
        # at the start line, move forward.
        elif gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[2] < GROUND_SENSOR_THRESHOLD:
            vL = MAX_SPEED
            vR = MAX_SPEED
            
        # d) Otherwise, if none of the ground sensors detect the line, rotate counterclockwise in place. (This
        # will help the robot re-find the line when it overshoots on corners!)     
        else: 
            vL = -1 *MAX_SPEED
            vR =  MAX_SPEED
    
    
    # TODO: Call update_odometry Here
    # pose_x, pose_y, pose_theta
    time = SIM_TIMESTEP/1000 # time step in seconds
    
    phi_left = vL*time
    phi_right = vR*time

    # radius of robot
    d = 0.071
    r = d/2
    #diameter 71mm
    #height 50mm
    #wheel radius 20.5mm
    #axle length 52mm
    
    theta_dot = r*phi_left/d + r*phi_right/d
    theta_delta = theta_dot*time

    x_dot_r = r*phi_left/2 + r*phi_right/2

    x_dot_i = math.cos(theta_delta)*x_dot_r
    y_dot_i = math.sin(theta_delta)*x_dot_r

    pose_x += x_dot_i*time
    pose_y += y_dot_i*time
    pose_theta += theta_delta

    # time=delta t in 3.35
    # x_dot is max speed and y_dot is 0 if moving on x axis, opposite for y axis
    # get the theta for each time step using 3.23 & 3.24


    # Hints:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second. 
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)

    

    
    # TODO: Insert Loop Closure Code Here
    is_ground = gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[2] < GROUND_SENSOR_THRESHOLD

    if(encountered_line and is_ground): 
        # reset
        pose_theta = 0.0
        pose_x = 0.0
        pose_y = 0.0
    elif (is_ground and not encountered_line): 
        encountered_line = True
    else: 
        encountered_line = False
    
    # Hints:
    #
    # 1) Set a flag whenever you encounter the line
    #
    # 2) Use the pose when you encounter the line last 
    # for best results
    
    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)