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
leftMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
rightMotor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
armMotor = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
forkMotor = Motor(Ports.PORT6, GearSetting.RATIO_18_1, True)

armMotor.set_stopping(HOLD)

# Initialize vision system
Vision3__FRUIT = Signature(1, -6429, -5309, -5869, -3997, -3303, -3650, 2.5, 0)
Vision3 = Vision(Ports.PORT20, 50, Vision3__FRUIT)

# Target fruit parameters
TARGET_X = 160  # X-coordinate to align with fruit
K_X = 0.5       # Proportional control constant for turning

# Line-following thresholds
lowerBound = 18
upperBound = 27

# Robot-specific constants
WHEEL_CIRCUMFERENCE_MM = 125.6  # Adjust this to match your wheel circumference
DISTANCE_TARGET_MM = 300       # Distance to drive before stopping

# Test results
detection_success = 0
collection_success = 0

# Drive function
def drive(left_speed, right_speed, duration=None):
    """
    Controls the left and right motors.
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
    Drives the robot a specific distance in millimeters and stops.
    """
    target_rotations = (target_mm / WHEEL_CIRCUMFERENCE_MM) * 360  # Degrees

    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)

    while True:
        left_position = abs(leftMotor.position(DEGREES))
        right_position = abs(rightMotor.position(DEGREES))

        if left_position < target_rotations and right_position < target_rotations:
            drive(30, 30)
        else:
            drive(0, 0)
            return True

# Line-following function
def line_follow():
    left_reflectivity = leftLine.reflectivity()
    right_reflectivity = rightLine.reflectivity()

    if lowerBound < left_reflectivity < upperBound and lowerBound < right_reflectivity < upperBound:
        print("Moving Straight")
        drive(30, -30)
    elif lowerBound < left_reflectivity < upperBound:
        print("Adjusting Left")
        drive(10, -30)
    elif lowerBound < right_reflectivity < upperBound:
        print("Adjusting Right")
        drive(30, -10)
    else:
        print("No Line Detected")
        drive(0, 0)

# Function to detect fruit with height filtering
def detect_fruit():
    objects = Vision3.take_snapshot(Vision3__FRUIT)
    if objects:
        largest = Vision3.largest_object()
        if 50 < largest.centerY < 100:  # Filter by expected height
            return largest.centerX, largest.centerY
    return None, None

# Function to log test results
def log_result(detected, collected):
    global detection_success, collection_success
    if detected:
        detection_success += 1
    if collected:
        collection_success += 1
    print(f"Detection Success: {detection_success}/20, Collection Success: {collection_success}/20")

# Main autonomous function
def mainFunction():
    global currentState, detection_success, collection_success

    if currentState == IDLE:
        print("State: IDLE")
        currentState = LINE_FOLLOWING

    elif currentState == LINE_FOLLOWING:
        print("State: LINE FOLLOWING")
        if drive_distance(DISTANCE_TARGET_MM):
            currentState = SEARCHING

    elif currentState == SEARCHING:
        print("State: SEARCHING")
        search_start_time = brain.timer.time(SECONDS)
        while brain.timer.time(SECONDS) - search_start_time < 5:  # 5-second timeout
            drive(-30, 30)  # Rotate to search for fruit
            cx, cy = detect_fruit()
            if cx is not None:
                print("Fruit detected, transitioning to APPROACHING")
                currentState = APPROACHING
                break
        else:
            print("Timeout reached, returning to LINE FOLLOWING")
            currentState = LINE_FOLLOWING

    elif currentState == APPROACHING:
        print("State: APPROACHING")
        cx, cy = detect_fruit()
        if cx is None:
            print("Lost fruit, returning to LINE FOLLOWING")
            currentState = LINE_FOLLOWING
        else:
            error = cx - TARGET_X
            turn_effort = K_X * error
            drive(20 + turn_effort, 20 - turn_effort)

            distance = ultraSonic.distance(MM)
            if distance < 150:
                print("Close to fruit, transitioning to COLLECTING")
                currentState = COLLECTING

    elif currentState == COLLECTING:
        print("State: COLLECTING")
        drive(0, 0)

        armMotor.spin_to_position(45, DEGREES)
        forkMotor.spin_to_position(30, DEGREES)
        wait(2, SECONDS)
        forkMotor.spin_to_position(0, DEGREES)
        armMotor.spin_to_position(0, DEGREES)

        currentState = LINE_FOLLOWING

# Run the autonomous loop
while True:
    mainFunction()
    wait(0.1, SECONDS)
