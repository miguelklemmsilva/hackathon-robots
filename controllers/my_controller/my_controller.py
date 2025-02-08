# controllers/main_controller.py

#!/opt/homebrew/bin/python3
"""
Main Controller for Spot Autonomous Deployment and Data Collection in Webots.
"""

from controller import Robot
import sys

# Import configuration and logging.
from utils.config import TIME_STEP, MOTOR_NAMES, CAMERA_NAMES, LED_NAMES, LIDAR_NAME
from utils.logger import setup_logger

# Import controllers and sensor modules.
from controllers.posture_controller import PostureController
from controllers.motion_controller import LocomotionController
from controllers.mission_planner import MissionPlanner
from sensors.lidar_sensor import LidarSensor
from sensors.camera_sensor import CameraSensor

# Set up logger.
logger = setup_logger('spot', 'spot_autonomous.log')


def main():
    # Create the robot instance.
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    # Get devices.
    motors = [robot.getDevice(name) for name in MOTOR_NAMES]
    cameras = [robot.getDevice(name) for name in CAMERA_NAMES]
    leds = [robot.getDevice(name) for name in LED_NAMES]
    for led in leds:
        led.set(1)

    # Define a simulation step callback.
    def step():
        if robot.step(time_step) == -1:
            sys.exit(0)

    # Instantiate sensor modules.
    lidar_sensor = LidarSensor(robot, LIDAR_NAME, time_step, logger=logger)
    camera_sensors = {name: CameraSensor(
        robot, name, time_step, logger=logger) for name in CAMERA_NAMES}

    # Instantiate controllers.
    posture_controller = PostureController(motors, time_step, step)
    locomotion_controller = LocomotionController(robot, time_step, step)
    mission_planner = MissionPlanner(
        lidar_sensor, locomotion_controller, logger)

    # Begin by standing up.
    posture_controller.stand_up(4.0)
    logger.info("Robot stood up and is ready for autonomous navigation.")

    # Main control loop.
    while robot.step(time_step) != -1:
        # Run the mission planner update.
        mission_planner.update()

        # Process camera image (using the left head camera as an example).
        left_camera = camera_sensors.get("left head camera")
        if left_camera:
            _ = left_camera.process_image()
            # Optionally, save or further process the image.
            # e.g., cv2.imwrite('latest_image.png', processed_image)


if __name__ == '__main__':
    main()
