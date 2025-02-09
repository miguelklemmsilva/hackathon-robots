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

        # Define safety thresholds for obstacle detection
        self.critical_distance = 1.0  # Immediate stop/turn needed
        self.warning_distance = 2.0   # Start planning avoidance
        # Weight factors for left, center, right
        self.sector_weights = [0.8, 1.0, 0.8]

        # Add filtering for more stable readings
        self.history_size = 3
        self.distance_history = []

    def get_range_image(self):
        return self.lidar.getRangeImage() if self.lidar else []

    def get_min_distance(self):
        ranges = self.get_range_image()
        return min(ranges) if ranges else None

    def get_filtered_ranges(self):
        """Get filtered LiDAR ranges to reduce noise"""
        current_ranges = self.get_range_image()
        if not current_ranges:
            return []

        self.distance_history.append(current_ranges)
        if len(self.distance_history) > self.history_size:
            self.distance_history.pop(0)

        # Use median filtering for stability
        filtered_ranges = []
        for i in range(len(current_ranges)):
            values = [ranges[i] for ranges in self.distance_history]
            filtered_ranges.append(sorted(values)[len(values)//2])

        return filtered_ranges

    def detect_obstacles(self):
        """
        Analyze LiDAR data to detect obstacles in different sectors.
        Returns: Dictionary with obstacle information for left, center, and right sectors
        """
        ranges = self.get_filtered_ranges()
        if not ranges:
            return {'left': False, 'center': False, 'right': False}

        # Divide the LiDAR data into three sectors with overlap
        sector_size = len(ranges) // 3
        overlap = sector_size // 3  # 25% overlap between sectors

        sectors = {
            'left': ranges[:sector_size + overlap],
            'center': ranges[sector_size - overlap:2*sector_size + overlap],
            'right': ranges[2*sector_size - overlap:]
        }

        # Calculate weighted minimum distances for each sector
        result = {}
        for i, (sector_name, sector_data) in enumerate(sectors.items()):
            # Use the minimum of the closest few points instead of absolute minimum
            closest_points = sorted(sector_data)[:3]
            min_dist = sum(closest_points) / \
                len(closest_points) * self.sector_weights[i]
            result[sector_name] = min_dist < self.warning_distance

            if min_dist < self.critical_distance:
                self.logger.warning(
                    f"Critical obstacle detected in {sector_name} sector!")

        return result

    def get_sector_distances(self):
        """
        Get the minimum distance to obstacles in each sector
        Returns: Dictionary with minimum distances for each sector
        """
        ranges = self.get_range_image()
        if not ranges:
            return {'left': float('inf'), 'center': float('inf'), 'right': float('inf')}

        sector_size = len(ranges) // 3
        return {
            'left': min(ranges[:sector_size]),
            'center': min(ranges[sector_size:2*sector_size]),
            'right': min(ranges[2*sector_size:])
        }
