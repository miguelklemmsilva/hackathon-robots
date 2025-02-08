# controllers/advanced_mission_planner.py
import math
from utils.config import K_ATTRACTIVE, K_REPULSIVE, REPULSIVE_RANGE


class MissionPlanner:
    def __init__(self, gps_sensor, inertial_unit_sensor, lidar_sensor, locomotion_controller, logger):
        self.gps_sensor = gps_sensor
        self.inertial_unit_sensor = inertial_unit_sensor
        self.lidar_sensor = lidar_sensor
        self.locomotion = locomotion_controller
        self.logger = logger

        # Initialize patrol variables
        self.patrol_points = []  # List of (x, z) coordinates to patrol
        self.current_goal_index = 0
        self.waypoint_threshold = 2.  # Distance in meters to consider waypoint reached

    def set_patrol_path(self, points):
        """Set a list of points for the robot to patrol.

        Args:
            points (list): List of (x, z) tuples representing patrol waypoints
        """
        self.patrol_points = points
        self.current_goal_index = 0
        self.logger.info(f"New patrol path set with {len(points)} waypoints")

    def get_current_goal(self):
        """Get the current waypoint the robot is moving towards."""
        if not self.patrol_points:
            return None
        return self.patrol_points[self.current_goal_index]

    def advance_to_next_waypoint(self):
        """Move to the next waypoint in the patrol sequence."""
        self.current_goal_index = (
            self.current_goal_index + 1) % len(self.patrol_points)
        self.logger.info(
            f"Moving to waypoint {self.current_goal_index}: {self.get_current_goal()}")

    def compute_attractive_force(self, current_pos):
        goal = self.get_current_goal()
        if goal is None:
            return (0.0, 0.0)
        dx = goal[0] - current_pos[0]
        dz = goal[1] - current_pos[1]
        return (K_ATTRACTIVE * dx, K_ATTRACTIVE * dz)

    def compute_repulsive_force(self):
        ranges = self.lidar_sensor.get_range_image()
        if not ranges:
            return (0.0, 0.0)
        num_rays = len(ranges)
        total_fx = 0.0
        total_fz = 0.0
        angle_increment = 2 * math.pi / num_rays  # assume 360° LiDAR
        for i, r in enumerate(ranges):
            if r < REPULSIVE_RANGE:
                angle = i * angle_increment
                force = K_REPULSIVE * (1.0 / r - 1.0 / REPULSIVE_RANGE)
                total_fx += force * math.cos(angle)
                total_fz += force * math.sin(angle)
        return (total_fx, total_fz)

    def compute_desired_heading(self, current_pos):
        if not self.patrol_points:
            return 0.0

        att_fx, att_fz = self.compute_attractive_force(current_pos)
        rep_fx, rep_fz = self.compute_repulsive_force()
        total_fx = att_fx - rep_fx
        total_fz = att_fz - rep_fz
        return math.atan2(total_fz, total_fx)

    def check_waypoint_reached(self, current_pos):
        """Check if we've reached the current waypoint."""
        goal = self.get_current_goal()
        if goal is None:
            return False

        dx = goal[0] - current_pos[0]
        dz = goal[1] - current_pos[1]
        distance = math.sqrt(dx*dx + dz*dz)
        return distance < self.waypoint_threshold

    def update(self):
        gps_values = self.gps_sensor.get_values()
        if gps_values is None:
            self.logger.warning("No GPS data available.")
            return

        current_pos = (gps_values[0], gps_values[2])

        # Check if we've reached the current waypoint
        if self.check_waypoint_reached(current_pos):
            self.advance_to_next_waypoint()

        desired_heading = self.compute_desired_heading(current_pos)
        current_heading = self.inertial_unit_sensor.get_heading()
        heading_error = desired_heading - current_heading

        # Normalize error to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        heading_tolerance = 0.2  # about 11°
        if abs(heading_error) > heading_tolerance:
            # Rotate to align with desired heading
            rotation_direction = 1 if heading_error > 0 else -1
            # Added turn rate multiplier
            self.locomotion.set_rotation(rotation_direction, 1.0)
        else:
            # Move forward with a slight turn to maintain heading
            turn_correction = heading_error / heading_tolerance  # normalized correction
            # Increased step velocity
            self.locomotion.set_gait(5.0, turn_correction, 1.0)

        self.logger.info(f"Current pos: {current_pos}, Goal: {self.get_current_goal()}, " +
                         f"Heading error: {heading_error:.2f}")
