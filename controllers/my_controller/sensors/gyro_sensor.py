# sensors/gyro_sensor.py
class GyroSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger
        self.gyro = self.robot.getDevice(self.device_name)
        if self.gyro:
            self.gyro.enable(self.time_step)
            if self.logger:
                self.logger.info(f"Gyro '{self.device_name}' enabled.")
        else:
            if self.logger:
                self.logger.warning(f"Gyro '{self.device_name}' not found.")

    def get_angular_velocity(self):
        if self.gyro:
            return self.gyro.getValues()  # Returns angular velocity (x, y, z)
        return [0.0, 0.0, 0.0]