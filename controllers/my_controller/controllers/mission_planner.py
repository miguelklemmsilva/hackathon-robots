# controllers/mission_planner.py

from utils.config import OBSTACLE_THRESHOLD

class MissionPlanner:
    def __init__(self, lidar_sensor, locomotion_controller, logger):
        """
        lidar_sensor: Instance of LidarSensor.
        locomotion_controller: Instance of LocomotionController.
        logger: Logger instance.
        """
        self.lidar_sensor = lidar_sensor
        self.locomotion = locomotion_controller
        self.logger = logger
        self.state = "IDLE"  # Initial state

    def update(self):
        """Decide and execute actions based on sensor readings."""
        distance = self.lidar_sensor.get_min_distance()
        if distance is not None and distance < OBSTACLE_THRESHOLD:
            self.logger.info("Obstacle detected at {:.2f} m".format(distance))
            self.state = "AVOID_OBSTACLE"
        else:
            self.state = "MOVE_FORWARD"

        if self.state == "AVOID_OBSTACLE":
            self.logger.info("Rotating left to avoid obstacle.")
            self.locomotion.rotate_left(1.0)
        elif self.state == "MOVE_FORWARD":
            self.logger.info("Path clear. Moving forward.")
            self.locomotion.walk_forward(0.5)