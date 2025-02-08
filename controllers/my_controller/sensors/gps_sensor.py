# sensors/gps_sensor.py

class GPSSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger
        self.gps = self.robot.getDevice(self.device_name)
        if self.gps:
            self.gps.enable(self.time_step)
            if self.logger:
                self.logger.info("GPS sensor enabled.")
        else:
            if self.logger:
                self.logger.warning("GPS sensor not found.")

    def get_values(self):
        if self.gps:
            # Typically returns [x, y, z]. We use x and z.
            return self.gps.getValues()
        return None
