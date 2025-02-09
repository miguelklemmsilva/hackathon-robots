# sensors/lidar_sensor.py

class LidarSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        """
        Initialize LiDAR sensor.

        Args:
            robot: Webots Robot instance
            device_name: LiDAR device name
            time_step: Simulation time step
            logger: Logger instance
        """
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger

        # Initialize LiDAR device
        self.lidar = self.robot.getDevice(self.device_name)
        if self.lidar is not None:
            self.lidar.enable(self.time_step)
            if self.logger:
                self.logger.info(f"LiDAR sensor enabled.")
        else:
            if self.logger:
                self.logger.warning(f"LiDAR sensor not found.")

        # Define safety thresholds for obstacle detection
        self.critical_distance = 1.0  # Immediate stop/turn needed
        self.warning_distance = 2.0   # Start planning avoidance

        # Weight factors for different sectors with wider overlap
        self.sector_weights = {
            'left': 0.8,
            'center': 1.0,
            'right': 0.8
        }

        # Add filtering for more stable readings
        self.history_size = 3
        self.distance_history = []

        # Add state for hysteresis
        self._last_detection = {}

    def get_range_image(self):
        """Get range data from LiDAR"""
        return self.lidar.getRangeImage() if self.lidar else []

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
        Analyze LiDAR data to detect obstacles with wide overlapping sectors.
        Returns: Dictionary with obstacle information for all sectors
        """
        ranges = self.get_filtered_ranges()
        if not ranges:
            return {'left': False, 'center': False, 'right': False}

        # Divide the LiDAR data into three sectors with significant overlap
        total_points = len(ranges)
        sector_size = total_points // 2  # Larger sectors for more overlap

        # Define sectors with significant overlap
        sectors = {
            'left': ranges[:sector_size],  # First half
            'center': ranges[total_points//4:3*total_points//4],  # Middle half
            'right': ranges[-sector_size:]  # Last half
        }

        result = {}
        for sector_name, sector_data in sectors.items():
            # Use more points for averaging
            # Increased from 5 to 7 points
            closest_points = sorted(sector_data)[:7]
            min_dist = sum(closest_points) / len(closest_points) * \
                self.sector_weights[sector_name]

            # Apply hysteresis
            if sector_name in self._last_detection:
                if self._last_detection[sector_name]:
                    threshold = self.warning_distance * 1.2  # Higher threshold to clear detection
                else:
                    threshold = self.warning_distance * 0.8  # Lower threshold to detect obstacle
            else:
                threshold = self.warning_distance

            result[sector_name] = min_dist < threshold

            if min_dist < self.critical_distance:
                self.logger.warning(
                    f"Critical obstacle detected in {sector_name} sector!")

        self._last_detection = result.copy()
        return result

    def get_sector_distances(self):
        """
        Get the minimum distance to obstacles in each sector
        Returns: Dictionary with minimum distances for each sector
        """
        ranges = self.get_range_image()
        if not ranges:
            return {'left': float('inf'), 'center': float('inf'), 'right': float('inf')}

        total_points = len(ranges)
        sector_size = total_points // 2

        # Use the same overlapping sectors as detect_obstacles
        return {
            'left': min(ranges[:sector_size]),
            'center': min(ranges[total_points//4:3*total_points//4]),
            'right': min(ranges[-sector_size:])
        }
