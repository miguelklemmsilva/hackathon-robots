#!/opt/homebrew/bin/python3
"""
        * Copyright 1996 -
    2024 Cyberbotics Ltd.
        *
            *Licensed under the Apache License,
    Version 2.0(the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Description:   Simple controller to present the Spot robot using Python.
 """

from controller import Robot
import math
import sys

#Constants
NUMBER_OF_LEDS = 8
NUMBER_OF_JOINTS = 12
NUMBER_OF_CAMERAS = 5

#Device names
motor_names = [
    "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
    "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
    "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
    "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"
]

camera_names = [
    "left head camera", "right head camera", "left flank camera",
    "right flank camera", "rear camera"
]

led_names = [
    "left top led", "left middle up led", "left middle down led",
    "left bottom led", "right top led", "right middle up led",
    "right middle down led", "right bottom led"
]

#Create the Robot instance.
robot = Robot()

#Get the simulation basic time step in milliseconds.
time_step = int(robot.getBasicTimeStep())

#Get cameras
cameras = [robot.getDevice(name) for name in camera_names]

#Enable the two front cameras.
cameras[0].enable(2 * time_step)
cameras[1].enable(2 * time_step)

#Get and turn on the LEDs.
leds = [robot.getDevice(name) for name in led_names]
for led in leds:
    led.set(1)

#Get motors(joints)
motors = [robot.getDevice(name) for name in motor_names]

#Utility function for simulation steps.
def step():
    if robot.step(time_step) == -1:
        sys.exit(0)

#Movement decomposition : gradually move all joints to the target positions over 'duration' seconds.
def movement_decomposition(target, duration):
    n_steps = int(duration * 1000 / time_step)
#Retrieve current target positions of the motors.
    current_position = [motor.getTargetPosition() for motor in motors]
#Compute the incremental change for each motor.
    step_difference = [(target[i] - current_position[i]) / n_steps for i in range(NUMBER_OF_JOINTS)]
    
    for _ in range(n_steps):
        for j in range(NUMBER_OF_JOINTS):
            current_position[j] += step_difference[j]
            motors[j].setPosition(current_position[j])
        step()

#Define motions
def lie_down(duration):
#Motor positions for lying down posture.
    motors_target_pos = [
        -0.40, -0.99, 1.59,   # Front left leg
         0.40, -0.99, 1.59,   # Front right leg
        -0.40, -0.99, 1.59,   # Rear left leg
         0.40, -0.99, 1.59    # Rear right leg
    ]
    movement_decomposition(motors_target_pos, duration)

def stand_up(duration):
#Motor positions for standing up posture.
    motors_target_pos = [
        -0.1, 0.0, 0.0,   # Front left leg
         0.1, 0.0, 0.0,   # Front right leg
        -0.1, 0.0, 0.0,   # Rear left leg
         0.1, 0.0, 0.0    # Rear right leg
    ]
    movement_decomposition(motors_target_pos, duration)

def sit_down(duration):
#Motor positions for sitting down posture.
    motors_target_pos = [
        -0.20, -0.40, -0.19,  # Front left leg
         0.20, -0.40, -0.19,  # Front right leg
        -0.40, -0.90, 1.18,   # Rear left leg
         0.40, -0.90, 1.18    # Rear right leg
    ]
    movement_decomposition(motors_target_pos, duration)

def give_paw():
#Step 1 : Stabilize posture
    motors_target_pos_1 = [
        -0.20, -0.30, 0.05,   # Front left leg
         0.20, -0.40, -0.19,   # Front right leg
        -0.40, -0.90, 1.18,    # Rear left leg
         0.49, -0.90, 0.80     # Rear right leg
    ]
    movement_decomposition(motors_target_pos_1, 4.0)

#Step 2 : Perform the "paw" movement for 8 seconds.
    start_time = robot.getTime()
    while robot.getTime() - start_time < 8:
#Adjust the movement of the motors corresponding to the paw(indices 4 and 5 in this case).
        motors[4].setPosition(0.2 * math.sin(2 * robot.getTime()) + 0.6)  # Upper arm movement.
        motors[5].setPosition(0.4 * math.sin(2 * robot.getTime()))         # Forearm movement.
        step()

#Step 3 : Return to sitting posture.
    motors_target_pos_2 = [
        -0.20, -0.40, -0.19,   # Front left leg
         0.20, -0.40, -0.19,   # Front right leg
        -0.40, -0.90, 1.18,    # Rear left leg
         0.40, -0.90, 1.18     # Rear right leg
    ]
    movement_decomposition(motors_target_pos_2, 4.0)

#Main control loop.
while 1:
    lie_down(4.0)
    stand_up(4.0)
    # sit_down(4.0)
    # give_paw()
    # stand_up(4.0)
    # lie_down(3.0)
    # stand_up(3.0)
    # lie_down(2.0)
    # stand_up(2.0)
    # lie_down(1.0)
    # stand_up(1.0)
    # lie_down(0.75)
    # stand_up(0.75)
    # lie_down(0.5)
    # stand_up(0.5)