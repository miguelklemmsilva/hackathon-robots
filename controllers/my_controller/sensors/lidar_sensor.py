# sensors/lidar_sensor.py

class LidarSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger
        self.lidar = self.robot.getDevice(self.device_name)
        if self.lidar is not None:
            self.lidar.enable(self.time_step)
            if self.logger:
                self.logger.info("LiDAR sensor enabled.")
        else:
            if self.logger:
                self.logger.warning("LiDAR sensor not found.")

    def get_range_image(self):
        return self.lidar.getRangeImage() if self.lidar else []

    def get_min_distance(self):
        ranges = self.get_range_image()
        return min(ranges) if ranges else None
