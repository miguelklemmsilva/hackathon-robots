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

    def get_heading_callback():
        return inertial_unit_sensor.get_heading()

    posture_controller = PostureController(motors, time_step, step)
    locomotion_controller = LocomotionController(
        robot, motors, time_step, step, get_heading_callback)
    mission_planner = MissionPlanner(
        gps_sensor, inertial_unit_sensor, lidar_sensor, locomotion_controller, logger)

    # Start with a standing posture.
    posture_controller.stand_up(4.0)
    logger.info("Robot stood up and is ready for autonomous navigation.")


    while robot.step(time_step) != -1:
        locomotion_controller.set_gait("walk", 0.1)
        locomotion_controller.update()

if __name__ == '__main__':
    main()
