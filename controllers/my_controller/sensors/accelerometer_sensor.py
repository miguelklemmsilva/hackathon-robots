# sensors/accelerometer_sensor.py
class AccelerometerSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger
        self.accelerometer = self.robot.getDevice(self.device_name)
        if self.accelerometer:
            self.accelerometer.enable(self.time_step)
            if self.logger:
                self.logger.info(f"Accelerometer '{self.device_name}' enabled.")
        else:
            if self.logger:
                self.logger.warning(f"Accelerometer '{self.device_name}' not found.")

    def get_acceleration(self):
        if self.accelerometer:
            x, y, z = self.accelerometer.getValues()
            return {"x": x, "y": y, "z": z}
        return {"x": 0.0, "y": 0.0, "z": 0.0}