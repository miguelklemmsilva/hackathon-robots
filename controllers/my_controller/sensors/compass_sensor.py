# sensors/compass_sensor.py
import numpy as np

class CompassSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger
        self.compass = self.robot.getDevice(self.device_name)
        if self.compass:
            self.compass.enable(self.time_step)
            if self.logger:
                self.logger.info(f"Compass '{self.device_name}' enabled.")
        else:
            if self.logger:
                self.logger.warning(f"Compass '{self.device_name}' not found.")

    def get_bearing(self):
        if self.compass:
            values = self.compass.getValues()
            heading = np.arctan2(values[0], values[2])
            return np.degrees(heading) if heading >= 0 else np.degrees(heading + 2 * np.pi)
        return 0.0