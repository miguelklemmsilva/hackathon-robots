# controllers/advanced_mission_planner.py
import math
from utils.config import GOAL_POSITION, K_ATTRACTIVE, K_REPULSIVE, REPULSIVE_RANGE


class MissionPlanner:
    def __init__(self, gps_sensor, inertial_unit_sensor, lidar_sensor, locomotion_controller, logger):
        self.gps_sensor = gps_sensor
        self.inertial_unit_sensor = inertial_unit_sensor
        self.lidar_sensor = lidar_sensor
        self.locomotion = locomotion_controller
        self.logger = logger
        self.goal = GOAL_POSITION  # (x, z) target

    def compute_attractive_force(self, current_pos):
        dx = self.goal[0] - current_pos[0]
        dz = self.goal[1] - current_pos[1]
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
        att_fx, att_fz = self.compute_attractive_force(current_pos)
        rep_fx, rep_fz = self.compute_repulsive_force()
        total_fx = att_fx - rep_fx
        total_fz = att_fz - rep_fz
        desired_heading = math.atan2(total_fz, total_fx)
        return desired_heading

    def update(self):
        gps_values = self.gps_sensor.get_values()
        if gps_values is None:
            self.logger.warning("No GPS data available.")
            return
        # We assume that the GPS returns (x, y, z); use x and z for horizontal motion.
        current_pos = (gps_values[0], gps_values[2])
        desired_heading = self.compute_desired_heading(current_pos)
        current_heading = self.inertial_unit_sensor.get_heading()
        heading_error = desired_heading - current_heading

        # Normalize error to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        self.logger.info("Desired heading: {:.2f} rad, Current heading: {:.2f} rad, Error: {:.2f}"
                         .format(desired_heading, current_heading, heading_error))

        heading_tolerance = 0.2  # about 11°
        if abs(heading_error) > heading_tolerance:
            self.logger.info("Rotating to align with desired heading.")
            # A short rotation command; the locomotion controller will update this each time step.
            if heading_error > 0:
                self.locomotion.set_gait("rotate", 0.1, {"direction": 1})
            else:
                self.locomotion.set_gait("rotate", 0.1, {"direction": -1})
        else:
            self.logger.info("Heading aligned. Driving forward.")
            self.locomotion.set_gait("walk", 0.1)
