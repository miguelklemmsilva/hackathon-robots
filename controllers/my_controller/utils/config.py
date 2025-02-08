# utils/config.py
TIME_STEP = 32  # Simulation time step in milliseconds

# Device configuration
NUMBER_OF_LEDS = 8
NUMBER_OF_JOINTS = 12
NUMBER_OF_CAMERAS = 5

MOTOR_NAMES = [
    "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
    "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
    "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
    "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"
]

CAMERA_NAMES = [
    "left head camera", "right head camera", "left flank camera",
    "right flank camera", "rear camera"
]

LED_NAMES = [
    "left top led", "left middle up led", "left middle down led",
    "left bottom led", "right top led", "right middle up led",
    "right middle down led", "right bottom led"
]

LIDAR_NAME = "Lidar"

# New sensors
GPS_NAME = "gps"
INERTIAL_UNIT_NAME = "inertial unit"

# Navigation and planning parameters
GOAL_POSITION = [5.0, 5.0]  # Target (x, z) coordinates (meters)
K_ATTRACTIVE = 1.0          # Gain for attractive force toward goal
K_REPULSIVE = 1.5           # Gain for repulsive force from obstacles
OBSTACLE_THRESHOLD = 1.0    # Minimum safe distance from obstacles (meters)
REPULSIVE_RANGE = 2.0       # Range within which obstacles repel (meters)

# Predefined postures (joint positions must match the motor grouping)
POSTURES = {
    "stand_up": [-0.1, 0.0, 0.0,
                 0.1,  0.0, 0.0,
                 -0.1, 0.0, 0.0,
                 0.1,  0.0, 0.0],
    "lie_down": [-0.40, -0.99, 1.59,
                 0.40, -0.99, 1.59,
                 -0.40, -0.99, 1.59,
                 0.40, -0.99, 1.59],
    "sit_down": [-0.20, -0.40, -0.19,
                 0.20, -0.40, -0.19,
                 -0.40, -0.90, 1.18,
                 0.40, -0.90, 1.18],
}
