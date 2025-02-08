# sensors/camera_sensor.py

import cv2
import numpy as np


class CameraSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        """
        robot: Webots Robot instance.
        device_name: Name of the camera device.
        time_step: Simulation time step (ms).
        logger: Optional logger instance.
        """
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger
        self.camera = self.robot.getDevice(self.device_name)
        if self.camera:
            self.camera.enable(2 * self.time_step)
            if self.logger:
                self.logger.info(f"Camera '{device_name}' enabled.")
        else:
            if self.logger:
                self.logger.warning(f"Camera '{device_name}' not found.")

    def get_image(self):
        return self.camera.getImage()

    def get_width(self):
        return self.camera.getWidth()

    def get_height(self):
        return self.camera.getHeight()

    def process_image(self):
        """
        Process the image to detect hazards.
        For example, detect high red intensity (which might indicate fire).
        """
        width = self.get_width()
        height = self.get_height()
        image = self.get_image()
        if image is None:
            return None
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        red_channel = img[:, :, 2]
        mean_red = np.mean(red_channel)
        if mean_red > 200:
            if self.logger:
                self.logger.info("Hazard detected: high red intensity!")
        return img
