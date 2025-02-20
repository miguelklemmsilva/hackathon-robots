#!/opt/homebrew/bin/python3
"""
Main Controller for Spot Autonomous Deployment and Data Collection in Webots.

This controller initializes the robot's devices and sensors, commands the robot
to assume a standing posture, and then demonstrates locomotion by switching
between in-place rotations (left, right, and none) using the LocomotionController API.
"""


import asyncio
import websockets
import json
from datetime import datetime, timezone
import numpy as np
from sensors.accelerometer_sensor import AccelerometerSensor
from sensors.compass_sensor import CompassSensor
from sensors.gyro_sensor import GyroSensor
from sensors.inertial_unit_sensor import InertialUnitSensor
from sensors.gps_sensor import GPSSensor
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
        d = math.sqrt((x - anchor[0])**2 +
                      (y - anchor[1])**2 + (z - anchor[2])**2)
        distances.append(d)
    return distances


def trilaterate(anchors, distances):
    """Solve for (x, y, z) given anchor positions and distances."""
    A = []
    b = []

    for i in range(len(anchors) - 1):
        x1, y1, z1 = anchors[i]
        x2, y2, z2 = anchors[-1]

        d1, d2 = distances[i], distances[-1]

        A.append([2 * (x1 - x2), 2 * (y1 - y2), 2 * (z1 - z2)])
        b.append(d1**2 - d2**2 - (x1**2 + y1**2 + z1**2) +
                 (x2**2 + y2**2 + z2**2))

    A = np.array(A)
    b = np.array(b)

    estimated_pos = np.linalg.lstsq(A, b, rcond=None)[0]
    return estimated_pos


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

WEBSOCKET_URL = "wss://lqq17im9ld.execute-api.eu-west-2.amazonaws.com/dev/"

# async def websocket_client(logger, gps_sensor, lidar_sensor, inertial_unit_sensor, accelerometer_sensor, gyro_sensor, compass_sensor):
#     """Handles WebSocket communication with AWS API Gateway."""
#     try:
#         async with websockets.connect(WEBSOCKET_URL) as ws:
#             logger.info(f"Connected to WebSocket server: {WEBSOCKET_URL}")

#             while True:
#                 # Collect sensor data
#                 gps_values = gps_sensor.get_values()
#                 lidar_min_distance = lidar_sensor.get_min_distance()
#                 heading = inertial_unit_sensor.get_heading()
#                 epoch_time = int(datetime.now(timezone.utc).timestamp())
#                 utc_datetime = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")

#                 sensor_data = {
#                     "type": "sensor_data",
#                     "timestamp": epoch_time,
#                     "gps": {"x": gps_values[0], "y": gps_values[1], "z": gps_values[2]},
#                     "lidar": {"min_distance": lidar_min_distance},
#                     "inertial_unit": {"heading": heading},
#                     # "accelerometer": {"x": acceleration[0], "y": acceleration[1], "z": acceleration[2]},
#                     # "gyro": {"angular_velocity_x": angular_velocity[0], "angular_velocity_y": angular_velocity[1], "angular_velocity_z": angular_velocity[2]},
#                 }

#                  # Prepare the message with "action" and "payload"
#                 message = {
#                     "action": "broadcast",
#                     "payload": sensor_data
#                 }

#                 message_size = len(json.dumps(message).encode("utf-8"))
#                 logger.info(f"Sending message of size: {message_size / 1024:.2f} KB")

#                 # Send the sensor data
#                 await ws.send(json.dumps(message))
#                 logger.info(f"Sent sensor data: {message}")

#                 # Wait for 5 seconds before sending the next sensor data
#                 await asyncio.sleep(5.0)

#     except Exception as e:
#         logger.error(f"WebSocket error: {e}")

async def websocket_client(logger, gps_sensor, lidar_sensor, inertial_unit_sensor, mission_planner):
    """Handles WebSocket communication with AWS API Gateway."""
    try:
        async with websockets.connect(WEBSOCKET_URL) as ws:
            logger.info(f"Connected to WebSocket server: {WEBSOCKET_URL}")

            async def send_sensor_data():
                """Send sensor data every 5 seconds."""
                while True:
                    gps_values = gps_sensor.get_values()
                    lidar_min_distance = lidar_sensor.get_min_distance()
                    heading = inertial_unit_sensor.get_heading()
                    epoch_time = int(datetime.now(timezone.utc).timestamp())

                    sensor_data = {
                        "type": "sensor_data",
                        "timestamp": epoch_time,
                        "gps": {"x": gps_values[0], "y": gps_values[1], "z": gps_values[2]},
                        "lidar": {"min_distance": lidar_min_distance},
                        "inertial_unit": {"heading": heading},
                    }

                    message = {"action": "broadcast", "payload": sensor_data}
                    await ws.send(json.dumps(message))
                    message_size = len(json.dumps(message).encode("utf-8"))
                    logger.info(f"Sent sensor data of size: {message_size / 1024:.2f} KB")

                    await asyncio.sleep(2.0)  # Send data every 5 seconds

            async def receive_move_commands(ws, logger, mission_planner):
                """Listen for incoming move commands and update the mission planner."""
                while True:
                    try:
                        message = await ws.recv()
                        logger.info(f"Received raw message: {message}")  # Log the raw message
                        parsed_message = json.loads(message)

                        if "payload" in parsed_message:
                            payload = parsed_message["payload"]

                            if payload.get("type") == "move_command":
                                coordinates = payload.get("coordinates", {})
                                x = coordinates.get("x")
                                y = coordinates.get("y")
                                if x is not None and y is not None:
                                    logger.info(f"Received move command: ({x}, {y})")
                                    mission_planner.set_patrol_path([(x, y)])
                                    mission_planner.logger.info(f"Overwriting patrol path to move to: ({x}, {y})")
                        else:
                            logger.warning("Received message with unknown action or missing payload.")

                    except websockets.exceptions.ConnectionClosed as e:
                        logger.error(f"WebSocket connection closed: {e}")
                        break
                    except Exception as e:
                        logger.error(f"Error receiving move command: {e}")
            # Run both tasks concurrently
            await asyncio.gather(send_sensor_data(), receive_move_commands(ws, logger, mission_planner))

    except Exception as e:
        logger.error(f"WebSocket error: {e}")

async def fire_detection_loop(logger, camera_sensors, leds):
    """Continuously detects fire without blocking movement."""
    while True:
        fire_detected = False  # Reset detection per loop

        for cam_name, cam_sensor in camera_sensors.items():
            logger.info(f"Looking for fire using {cam_name}...")
            frame, detected = cam_sensor.detect_fire()

            if detected:
                fire_detected = True
                logger.info(f"🔥 Fire detected by {cam_name}!")

        if fire_detected:
            # Blink LED lights without affecting movement
            for led in leds:
                led.set(0)
            await asyncio.sleep(0.1)  # Short delay
            for led in leds:
                led.set(1)
            await asyncio.sleep(0.1)

        # Run this check every ~0.5s
        await asyncio.sleep(0.5)

async def main():
    # Instantiate the Webots robot and determine the time step.
    print("ONNX Runtime is installed and working!")
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
    lidar_sensor = LidarSensor(robot, 'Lidar', time_step, logger=logger)
    camera_sensors = {name: CameraSensor(
        robot, name, time_step, logger=logger) for name in CAMERA_NAMES}
    gps_sensor = GPSSensor(robot, GPS_NAME, time_step, logger=logger)
    inertial_unit_sensor = InertialUnitSensor(
        robot, INERTIAL_UNIT_NAME, time_step, logger=logger)
    accelerometer_sensor = AccelerometerSensor(robot, "accelerometer", time_step, logger=logger)
    gyro_sensor = GyroSensor(robot, "gyro", time_step, logger=logger)
    compass_sensor = CompassSensor(robot, "compass", time_step, logger=logger)
        # Initialize controllers.
    posture_controller = PostureController(motors, time_step, step)
    locomotion_controller = LocomotionController(
        robot, motors, time_step, step, logger)
    mission_planner = MissionPlanner(
        gps_sensor, inertial_unit_sensor, lidar_sensor, locomotion_controller, logger)

    # Define patrol waypoints (x, z coordinates)
    patrol_points = [
        (7.18, 0.0),    # 5 meters forward
        (11.04, -4.11),    # 5 meters right
        (3.96, 7.65),    # 5 meters back
        (11.04, 11.84),    # 5 meters left
        (0.0, 0.0)
    ]

    # Set the patrol path
    mission_planner.set_patrol_path(patrol_points)

    # Begin with a standing posture
    posture_controller.stand_up(4.0)
    logger.info("Robot stood up and is ready for autonomous navigation.")
    locomotion_controller.set_gait(5.0, 0.0, 1.0)

    gps_values = gps_sensor.get_values()
    x, y, z = gps_values[0], gps_values[1], gps_values[2]
    distances = get_distances(x, y, z)

    logger.info(f"UWB Distances: {distances}")
    # Example usage:
    estimated_pos = trilaterate(anchors, distances)
    logger.info(f"Estimated Position: {estimated_pos}")

    # create an asyncio task for the websocket client
    websocket_task = asyncio.create_task(websocket_client(logger, gps_sensor, lidar_sensor, inertial_unit_sensor, mission_planner))

    # fire_detection_task = asyncio.create_task(fire_detection_loop(logger, camera_sensors, leds))


    # Main simulation loop
    while robot.step(time_step) != -1:
        # Get sensor data
        obstacles = lidar_sensor.detect_obstacles()
        sector_distances = lidar_sensor.get_sector_distances()

        # Check for obstacles and handle avoidance
        if any(obstacles.values()):
            if locomotion_controller.avoid_obstacle(obstacles):
                logger.info(f"Obstacle avoidance active. Distances - Left: {sector_distances['left']:.2f}m, "
                            f"Center: {sector_distances['center']:.2f}m, Right: {sector_distances['right']:.2f}m")
        elif locomotion_controller.avoiding_obstacle:
            # If no obstacles and we were avoiding, resume normal movement
            locomotion_controller.resume_normal_movement()
            logger.info("Path clear, resuming normal movement")

        # Only update mission planner if we're not actively avoiding obstacles
        if not locomotion_controller.avoiding_obstacle:
            mission_planner.update()

        # Update the locomotion controller to compute and apply motor positions
        locomotion_controller.update()

        # give control to the event loopto allow async tasks to run
        await asyncio.sleep(0)
    
    # close websocket task once simulation ends
    websocket_task.cancel()

    # fire_detection_task.cancel()

if __name__ == '__main__':
    # main()
    asyncio.run(main())