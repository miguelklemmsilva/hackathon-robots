#!/opt/homebrew/bin/python3
"""
Main Controller for Spot Autonomous Deployment and Data Collection in Webots.

This controller initializes the robot's devices and sensors, commands the robot
to assume a standing posture, and then demonstrates locomotion by switching
between in-place rotations (left, right, and none) using the LocomotionController API.
"""

from controller import Robot
import sys
from enum import Enum

from utils.config import TIME_STEP, MOTOR_NAMES, CAMERA_NAMES, LED_NAMES, LIDAR_NAME, GPS_NAME, INERTIAL_UNIT_NAME
from utils.logger import setup_logger

from controllers.posture_controller import PostureController
from controllers.locomotion_controller import LocomotionController
from controllers.mission_planner import MissionPlanner
from sensors.lidar_sensor import LidarSensor
from sensors.camera_sensor import CameraSensor
from sensors.gps_sensor import GPSSensor
from sensors.inertial_unit_sensor import InertialUnitSensor

# Define motor names for Spot's 12 joints.
MOTOR_NAMES = [
    "front left shoulder abduction motor",  # 0
    "front left shoulder rotation motor",   # 1
    "front left elbow motor",                 # 2
    "front right shoulder abduction motor",   # 3
    "front right shoulder rotation motor",    # 4
    "front right elbow motor",                # 5
    "rear left shoulder abduction motor",     # 6
    "rear left shoulder rotation motor",      # 7
    "rear left elbow motor",                  # 8
    "rear right shoulder abduction motor",    # 9
    "rear right shoulder rotation motor",     # 10
    "rear right elbow motor"                  # 11
]

# Define an enum for intuitive rotation directions.


class RotationDirection(Enum):
    LEFT = 1
    NONE = 0
    RIGHT = -1


def main():
    # Instantiate the Webots robot and determine the time step.
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    logger = setup_logger('spot', 'spot_autonomous.log')

    # Retrieve motor, camera, and LED devices.
    motors = [robot.getDevice(name) for name in MOTOR_NAMES]
    cameras = [robot.getDevice(name) for name in CAMERA_NAMES]
    leds = [robot.getDevice(name) for name in LED_NAMES]
    for led in leds:
        led.set(1)  # Activate LEDs

    # Define a step function to advance the simulation and handle exit.
    def step():
        if robot.step(time_step) == -1:
            sys.exit(0)

    # Initialize sensor interfaces.
    lidar_sensor = LidarSensor(robot, LIDAR_NAME, time_step, logger=logger)
    camera_sensors = {name: CameraSensor(
        robot, name, time_step, logger=logger) for name in CAMERA_NAMES}
    gps_sensor = GPSSensor(robot, GPS_NAME, time_step, logger=logger)
    inertial_unit_sensor = InertialUnitSensor(
        robot, INERTIAL_UNIT_NAME, time_step, logger=logger)

    # Initialize controllers.
    posture_controller = PostureController(motors, time_step, step)
    locomotion_controller = LocomotionController(
        robot, motors, time_step, step, logger)
    mission_planner = MissionPlanner(
        gps_sensor, inertial_unit_sensor, lidar_sensor, locomotion_controller, logger)

    # Define patrol waypoints (x, z coordinates)
    patrol_points = [
        (0.0, 5.0),    # 5 meters forward
        (5.0, 5.0),    # 5 meters right
        (5.0, 0.0),    # 5 meters back
        (0.0, 0.0)     # Return to start
    ]

    # Set the patrol path
    mission_planner.set_patrol_path(patrol_points)

    # Begin with a standing posture
    posture_controller.stand_up(4.0)
    logger.info("Robot stood up and is ready for autonomous navigation.")
    locomotion_controller.set_gait(5.0, 0.0, 1.0)

    # Main simulation loop
    while robot.step(time_step) != -1:
        # Update mission planner to handle navigation
        mission_planner.update()

        # Update the locomotion controller to compute and apply motor positions
        locomotion_controller.update()


if __name__ == '__main__':
    main()
