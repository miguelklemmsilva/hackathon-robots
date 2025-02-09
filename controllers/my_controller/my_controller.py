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
from sensors.inertial_unit_sensor import InertialUnitSensor
from sensors.gps_sensor import GPSSensor
from controller import Robot
import cv2
import sys
from enum import Enum

import os
print(f"Webots is using Python: {sys.executable}")
print("Checking model path:", os.path.exists("runs/detect/train18/weights/best.onnx"))
print("📂 Webots Working Directory:", os.getcwd())
print("📁 Looking for model at:", os.path.abspath("runs/detect/train18/weights/best.onnx"))
print("✅ Model Exists:", os.path.exists(os.path.abspath("runs/detect/train18/weights/best.onnx")))

try:
    import onnxruntime
    print("✅ ONNX Runtime is installed and working!")
except ModuleNotFoundError:
    print("❌ ONNX Runtime is NOT installed in this Python environment.")

import onnxruntime

from utils.config import TIME_STEP, MOTOR_NAMES, CAMERA_NAMES, LED_NAMES, LIDAR_NAME, GPS_NAME, INERTIAL_UNIT_NAME
from utils.logger import setup_logger

from controllers.posture_controller import PostureController
from controllers.locomotion_controller import LocomotionController
from controllers.mission_planner import MissionPlanner
from sensors.lidar_sensor import LidarSensor
from sensors.camera_sensor import CameraSensor


import math

import onnxruntime
print("ONNX Runtime is installed and working!")

print("hello whatsup")
print(sys.executable)

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

async def websocket_client(logger):
    """Handles WebSocket communication with AWS API Gateway."""
    try:
        async with websockets.connect(WEBSOCKET_URL) as ws:
            logger.info(f"Connected to WebSocket server: {WEBSOCKET_URL}")
            
            while True:
                # Send a message periodically
                utc_datetime = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
                message = {
                    "action": "broadcast",
                    "message": f"message at timestamp: {utc_datetime}"
                }
                await ws.send(json.dumps(message))
                logger.info(f"Sent message: {message}")

                # Wait for a response (optional)
                try:
                    response = await asyncio.wait_for(ws.recv(), timeout=5.0)
                    logger.info(f"Received response: {response}")
                except asyncio.TimeoutError:
                    logger.warning("No response from WebSocket server.")

                # Wait for a 30 seconds before sending the next message
                await asyncio.sleep(30)

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
        (10.0, 0.0),    # 5 meters back
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
    websocket_task = asyncio.create_task(websocket_client(logger))

    fire_detection_task = asyncio.create_task(fire_detection_loop(logger, camera_sensors, leds))


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

    fire_detection_task.cancel()

if __name__ == '__main__':
    # main()
    asyncio.run(main())