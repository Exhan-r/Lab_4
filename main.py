# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       ethanramoth                                                  #
# 	Created:      11/26/2024, 2:07:37 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Constants for states
IDLE = 0
SEARCHING = 1
APPROACHING = 2
COLLECTING = 3
LINE_FOLLOWING = 4

# Definitions
brain = Brain()
currentState = IDLE

# Initialize ultrasonic sensor for distance
ultraSonic = Sonar(brain.three_wire_port.g)

# Line tracker sensors
leftLine = Line(brain.three_wire_port.f)
rightLine = Line(brain.three_wire_port.e)

# Motors
leftMotor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
armMotor = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)
forkMotor = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

armMotor.set_stopping(HOLD)

# Initialize vision system
Vision3__FRUIT = Signature(1, -6429, -5309, -5869, -3997, -3303, -3650, 2.5, 0)
Vision3 = Vision(Ports.PORT16, 50, Vision3__FRUIT)

# Target fruit parameters
TARGET_X = 160  # X-coordinate to align with fruit
K_X = 0.5       # Proportional control constant for turning

# Line-following thresholds
lowerBound = 18
upperBound = 27

# Robot-specific constants
WHEEL_CIRCUMFERENCE_MM = 125.6  # Adjust this to match your wheel circumference
DISTANCE_TARGET_MM = 300      # Distance to drive before stopping

# Drive function
def drive(left_speed, right_speed, duration=None):
    """
    Controls the left and right motors.
    :param left_speed: Speed for the left motor (percentage).
    :param right_speed: Speed for the right motor (percentage).
    :param duration: Optional duration (seconds) to run the motors.
    """
    leftMotor.spin(FORWARD, left_speed, PERCENT)
    rightMotor.spin(FORWARD, right_speed, PERCENT)
    if duration:
        wait(duration, SECONDS)
        leftMotor.stop()
        rightMotor.stop()

# Function to drive a specific distance
def drive_distance(target_mm):
    """
    Drives the robot a specific distance in millimeters.
    :param target_mm: Distance in millimeters to drive.
    """
    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)
    
    target_rotations = (target_mm / WHEEL_CIRCUMFERENCE_MM) * 360  # Degrees to rotate
    
    # Drive until the target distance is reached
    if abs(leftMotor.position(DEGREES)) < target_rotations and abs(rightMotor.position(DEGREES)) < target_rotations:
        drive(-30, -30)
        print()
        return False
    else:
        drive(0, 0)  # Stop the robot after driving
        return True

# Line-following function with upper and lower bounds
def line_follow():
    """
    Follows a line using two line sensors and upper/lower bounds for reflectivity.
    """
    left_reflectivity = leftLine.reflectivity()
    right_reflectivity = rightLine.reflectivity()

    if lowerBound < left_reflectivity < upperBound and lowerBound < right_reflectivity < upperBound:
        # Both sensors detect the line, move straight
        print("Moving Straight")
        drive(30, -30)
    elif lowerBound < left_reflectivity < upperBound:
        # Adjust left
        print("Adjusting Left")
        drive(10, -30)
    elif lowerBound < right_reflectivity < upperBound:
        # Adjust right
        print("Adjusting Right")
        drive(30, -10)
    else:
        # No line detected, stop or search
        print("No Line Detected")
        drive(0, 0)

# Function to handle object detection
def detect_fruit():
    objects = Vision3.take_snapshot(Vision3__FRUIT)
    if objects:
        largest = Vision3.largest_object()
        return largest.centerX, largest.centerY
    return None, None

# Main autonomous function
def mainFunction():
    global currentState

    if currentState == IDLE:
        print("State: IDLE")
        # Transition to LINE_FOLLOWING to start moving along the orchard
        currentState = LINE_FOLLOWING

    elif currentState == LINE_FOLLOWING:
        print("State: LINE FOLLOWING")
        # Drive forward for 300 mm
    
        if drive_distance(DISTANCE_TARGET_MM) == True:
            
            currentState = SEARCHING  # Transition to searching after driving the set distance

    elif currentState == SEARCHING:
        print("State: SEARCHING")
        # Rotate in place to search for fruit
        drive(-30, 30)  # Rotate to search for the fruit

        # Detect fruit
        cx, cy = detect_fruit()
        if cx is not None:
            print("Fruit detected, transitioning to APPROACHING")
            currentState = APPROACHING

    elif currentState == APPROACHING:
        print("State: APPROACHING")
        # Detect fruit and adjust alignment
        cx, cy = detect_fruit()
        if cx is None:
            print("Lost fruit, returning to LINE FOLLOWING")
            currentState = LINE_FOLLOWING
        else:
            error = cx - TARGET_X
            turn_effort = K_X * error

            # Adjust direction and move forward
            drive(20 + turn_effort, 20 - turn_effort)

            # Check distance using ultrasonic sensor
            distance = ultraSonic.distance(MM)
            if distance < 150:  # Close enough to the fruit
                print("Close to fruit, transitioning to COLLECTING")
                currentState = COLLECTING

    elif currentState == COLLECTING:
        print("State: COLLECTING")
        # Stop motors
        drive(0, 0)

        # Move arm and fork to collect fruit
        armMotor.spin_to_position(45, DEGREES)  # Raise arm
        forkMotor.spin_to_position(30, DEGREES)  # Open fork
        wait(2, SECONDS)  # Simulate collection action
        forkMotor.spin_to_position(0, DEGREES)  # Close fork
        armMotor.spin_to_position(0, DEGREES)  # Lower arm

        print("Fruit collected, returning to LINE FOLLOWING")
        currentState = LINE_FOLLOWING

# Run the autonomous loop
while True:
    mainFunction()
    wait(0.1, SECONDS)
