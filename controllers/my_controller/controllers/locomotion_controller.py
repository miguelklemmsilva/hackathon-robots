import math
import numpy as np
from webots_spot.SpotKinematics import SpotModel
from webots_spot.Bezier import BezierGait
import copy


class LocomotionController:
    def __init__(self, robot, motors, time_step, step_callback):
        """
        robot: The Webots Robot instance.
        motors: List of motor devices.
        time_step: Simulation time step (in milliseconds).
        step_callback: Function to advance the simulation (e.g., a wrapper around robot.step).
        """
        self.robot = robot
        self.motors = motors
        self.time_step = time_step
        self.step = step_callback
        self.start_time = self.robot.getTime()

        # Initialize SpotModel and BezierGait
        self.spot = SpotModel()
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)
        self.bzg = BezierGait(dt=time_step/1000.0)  # Convert ms to seconds

        # Motor offsets for each joint type
        self.motor_offsets = {
            'abduction': 0.0,
            'rotation': 0.6,  # ~30 degrees
            'elbow': -1.19    # ~-67.7 degrees
        }

        # Gait parameters
        self.xd = 0.0  # Forward position
        self.yd = 0.0  # Lateral position
        self.zd = 0.0  # Height
        self.rolld = 0.0
        self.pitchd = 0.0
        self.yawd = 0.0

        # Gait control parameters
        self.step_length = 0.0
        self.lateral_fraction = 0.0
        self.yaw_rate = 0.0
        self.step_velocity = 0.0
        self.clearance_height = 0.04
        self.penetration_depth = 0.01
        self.swing_period = 0.2

        # Gait state
        self.gait_mode = "stand"  # "stand", "walk", or "rotate"
        self.rotation_direction = 0

    def set_gait(self, duration, rotation_direction=0):
        """
        Initiate a forward walk gait for the specified duration (seconds).
        rotation_direction: -1 for left turn, 0 for straight, 1 for right turn
        """
        self.gait_mode = "walk"
        self.step_length = 0.15  # Base step length
        self.step_velocity = 0.8
        self.rotation_direction = rotation_direction

        if rotation_direction != 0:
            self.yaw_rate = 0.3 * rotation_direction
            self.step_length *= 0.5  # Reduce step length during turns
        else:
            self.yaw_rate = 0.0


    def set_rotation(self, direction):
        """
        Set the robot to rotate in place.
        direction: -1 for left turn, 1 for right turn
        """
        self.gait_mode = "rotate"
        self.step_length = 0.05
        self.yaw_rate = 0.3 * direction
        self.rotation_direction = direction

    def apply_motor_offset(self, joint_idx, angle):
        """Apply the appropriate offset based on joint type"""
        joint_type = joint_idx % 3  # 0: abduction, 1: rotation, 2: elbow
        if joint_type == 0:
            return angle + self.motor_offsets['abduction']
        elif joint_type == 1:
            return angle + self.motor_offsets['rotation']
        else:  # elbow
            return angle + self.motor_offsets['elbow']

    def update(self):
        """Call this method every simulation step to update motor positions."""
        if self.gait_mode == "stand":
            self.set_standing_positions()
        else:
            # Get foot contact states (simplified)
            contacts = [1, 1, 1, 1]  # Assuming all feet in contact

            # Generate trajectory using BezierGait
            T_bf = self.bzg.GenerateTrajectory(
                self.step_length,
                self.lateral_fraction,
                self.yaw_rate,
                self.step_velocity,
                self.T_bf0,
                self.T_bf,
                self.clearance_height,
                self.penetration_depth,
                contacts
            )

            # Calculate joint angles using inverse kinematics
            orn = np.array([self.rolld, self.pitchd, self.yawd])
            pos = np.array([self.xd, self.yd, self.zd])
            joint_angles = -self.spot.IK(orn, pos, T_bf)

            # Set motor positions with offsets
            motor_positions = []
            for leg in joint_angles:
                motor_positions.extend(leg)

            for i, motor in enumerate(self.motors):
                motor.setPosition(
                    self.apply_motor_offset(i, motor_positions[i]))

        self.step()

    def set_standing_positions(self):
        """Place all joints in a stable standing posture."""
        # Base positions before offsets
        base_positions = [
            0.0, 0.0, 0.0,  # front left leg
            0.0, 0.0, 0.0,  # front right leg
            0.0, 0.0, 0.0,  # rear left leg
            0.0, 0.0, 0.0   # rear right leg
        ]

        # Apply offsets for each joint
        for i, pos in enumerate(base_positions):
            self.motors[i].setPosition(self.apply_motor_offset(i, pos))
