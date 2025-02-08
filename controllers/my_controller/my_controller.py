# controllers/main_controller.py
#!/opt/homebrew/bin/python3
"""
Main Controller for Spot Autonomous Deployment and Data Collection in Webots.
"""

from controller import Robot
import sys

from utils.config import TIME_STEP, MOTOR_NAMES, CAMERA_NAMES, LED_NAMES, LIDAR_NAME, GPS_NAME, INERTIAL_UNIT_NAME
from utils.logger import setup_logger

from controllers.posture_controller import PostureController
from controllers.locomotion_controller import LocomotionController
from controllers.mission_planner import MissionPlanner
from sensors.lidar_sensor import LidarSensor
from sensors.camera_sensor import CameraSensor


import math

# UWB anchor positions (x, y, z)
anchors = [
    (0, 0, 0),   # Anchor 1
    (5, 0, 0),   # Anchor 2
    (2.5, 0, 5)  # Anchor 3
]

def get_distances(x, y, z):
    """Calculate distances from Spot to each anchor."""
    distances = []
    for anchor in anchors:
        d = math.sqrt((x - anchor[0])**2 + (y - anchor[1])**2 + (z - anchor[2])**2)
        distances.append(d)
    return distances

import numpy as np

def trilaterate(anchors, distances):
    """Solve for (x, y, z) given anchor positions and distances."""
    A = []
    b = []

    for i in range(len(anchors) - 1):
        x1, y1, z1 = anchors[i]
        x2, y2, z2 = anchors[-1]

        d1, d2 = distances[i], distances[-1]
        
        A.append([2 * (x1 - x2), 2 * (y1 - y2), 2 * (z1 - z2)])
        b.append(d1**2 - d2**2 - (x1**2 + y1**2 + z1**2) + (x2**2 + y2**2 + z2**2))
    
    A = np.array(A)
    b = np.array(b)

    estimated_pos = np.linalg.lstsq(A, b, rcond=None)[0]
    return estimated_pos






from sensors.gps_sensor import GPSSensor
from sensors.inertial_unit_sensor import InertialUnitSensor

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


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    logger = setup_logger('spot', 'spot_autonomous.log')

    motors = [robot.getDevice(name) for name in MOTOR_NAMES]
    cameras = [robot.getDevice(name) for name in CAMERA_NAMES]
    leds = [robot.getDevice(name) for name in LED_NAMES]
    for led in leds:
        led.set(1)

    def step():
        if robot.step(time_step) == -1:
            sys.exit(0)

    lidar_sensor = LidarSensor(robot, LIDAR_NAME, time_step, logger=logger)
    camera_sensors = {name: CameraSensor(
        robot, name, time_step, logger=logger) for name in CAMERA_NAMES}
    gps_sensor = GPSSensor(robot, GPS_NAME, time_step, logger=logger)
    inertial_unit_sensor = InertialUnitSensor(
        robot, INERTIAL_UNIT_NAME, time_step, logger=logger)

    posture_controller = PostureController(motors, time_step, step)
    locomotion_controller = LocomotionController(
        robot, motors, time_step, step)
    mission_planner = MissionPlanner(
        gps_sensor, inertial_unit_sensor, lidar_sensor, locomotion_controller, logger)

    # Start with a standing posture.
    posture_controller.stand_up(4.0)
    logger.info("Robot stood up and is ready for autonomous navigation.")

    gps_values = gps_sensor.get_values()
    x, y, z = gps_values[0], gps_values[1], gps_values[2]
    distances = get_distances(x, y, z)

    logger.info(f"UWB Distances: {distances}")
    # Example usage:
    estimated_pos = trilaterate(anchors, distances)
    logger.info(f"Estimated Position: {estimated_pos}")


    gait_duration = 5.0
    locomotion_controller.set_gait(gait_duration, rotation_direction=0)

    # Main simulation loop.
    while robot.step(time_step) != -1:
        current_time = robot.getTime()

        if current_time < 5.0:
            # First 5 seconds: rotate left in place
            locomotion_controller.set_rotation(-1)
        elif current_time < 10.0:
            # Next 5 seconds: rotate right in place
            locomotion_controller.set_rotation(1)

        locomotion_controller.update()


if __name__ == '__main__':
    main()
