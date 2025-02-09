#!/usr/bin/env python3
"""
Locomotion Controller API for Spot Autonomous Deployment.

This module provides the LocomotionController class which is responsible for
managing the gait generation, motor command computation, and execution for the
Spot robot in the Webots simulation environment.
"""

import math
import numpy as np
import copy
from enum import Enum
from webots_spot.SpotKinematics import SpotModel
from webots_spot.Bezier import BezierGait

# Define an enum for intuitive rotation directions.


class RotationDirection(Enum):
    LEFT = -1
    NONE = 0
    RIGHT = 1


class LocomotionController:
    def __init__(self, robot, motors, time_step, step_callback, logger):
        """
        Initialize the locomotion controller.

        Parameters:
            robot (Robot): The Webots Robot instance.
            motors (list): List of motor devices.
            time_step (int): Simulation time step (in milliseconds).
            step_callback (callable): Function to advance the simulation (e.g., a wrapper around robot.step).
        """
        self.robot = robot
        self.motors = motors
        self.time_step = time_step
        self.step = step_callback
        self.start_time = self.robot.getTime()
        self.logger = logger

        # Initialize SpotModel and BezierGait for inverse kinematics and trajectory generation.
        self.spot = SpotModel()
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)
        self.bzg = BezierGait(dt=time_step / 1000.0)  # Convert ms to seconds

        # Motor offsets for each joint type (to compensate for mechanical configuration)
        self.motor_offsets = {
            'abduction': 0.0,
            'rotation': 0.55,  # ~30 degrees offset
            'elbow': -1.19    # ~-67.7 degrees offset
        }

        # Gait parameters (desired body pose and position adjustments)
        self.xd = 0.0  # Forward displacement
        self.yd = 0.0  # Lateral displacement
        self.zd = 0.0  # Vertical displacement (height)
        self.rolld = 0.0
        self.pitchd = 0.0
        self.yawd = 0.0

        # Gait control parameters (dynamics and trajectory shaping)
        self.step_length = 0.0
        self.lateral_fraction = 0.0
        self.yaw_rate = 0.0
        self.step_velocity = 0.0
        self.clearance_height = 0.04
        self.penetration_depth = 0.01
        self.swing_period = 0.2

        # Gait state: "stand", "walk", or "rotate"
        self.gait_mode = "stand"
        self.rotation_direction = 0

        # Enhanced obstacle avoidance state
        self.avoiding_obstacle = False
        self.avoidance_direction = None
        self.avoidance_start_time = None
        self.min_avoidance_time = 1.5  # Increased minimum turning time
        self.max_avoidance_time = 8.0  # Increased maximum avoidance time
        # Increased cooldown to prevent rapid switching
        self.post_avoidance_cooldown = 3.0
        self.last_avoidance_end_time = 0
        self.forward_recovery_time = 5.0  # Increased forward movement time
        self.forward_recovery_attempts = 0
        self.max_recovery_attempts = 3
        self.recovery_angle_threshold = math.pi / 5  # 30 degrees

    def set_gait(self, duration, rotation_direction=0, turn_rate_multiplier=1.0):
        """
        Initiate a forward walk gait for the specified duration.

        Parameters:
            duration (float): Duration of the gait in seconds.
            rotation_direction (RotationDirection or int): 
                Use RotationDirection.LEFT for left turn, 
                RotationDirection.NONE (or 0) for straight, 
                RotationDirection.RIGHT for right turn.
            turn_rate_multiplier (float): Controls how tight the turn is. 
                Default 1.0, higher values make tighter turns.
        """
        # Allow enum values or raw integers.
        if hasattr(rotation_direction, 'value'):
            rotation_value = rotation_direction.value
        else:
            rotation_value = rotation_direction

        self.gait_mode = "walk"
        self.step_length = 0.15  # Base step length
        self.step_velocity = 0.8
        self.rotation_direction = rotation_value

        if rotation_value != 0:
            self.yaw_rate = 0.6 * rotation_value * \
                turn_rate_multiplier  # Adjustable turning rate
            # Reduce step length and velocity during turns for stability.
            self.step_length *= 0.3
            self.step_velocity *= 0.7
        else:
            self.yaw_rate = 0.0

    def set_rotation(self, rotation, turn_rate_multiplier=1.0):
        """
        Set the robot to rotate in place using a rotation direction.

        Parameters:
            rotation (RotationDirection or int):
                Use RotationDirection.LEFT for left turn, 
                RotationDirection.NONE (or 0) for no rotation, 
                or RotationDirection.RIGHT for right turn.
            turn_rate_multiplier (float): Controls how tight the turn is.
                Default 1.0, higher values make tighter turns.
        """
        # Allow enum values or raw integers.
        if hasattr(rotation, 'value'):
            rotation_value = rotation.value
        else:
            rotation_value = rotation

        self.gait_mode = "rotate"
        self.step_length = 0.03  # Smaller step length for in-place rotation
        # Compute turning rate using the rotation value and multiplier
        self.yaw_rate = 0.6 * rotation_value * turn_rate_multiplier
        self.rotation_direction = rotation_value

    def apply_motor_offset(self, joint_idx, angle):
        """
        Apply the appropriate motor offset based on joint type.

        Parameters:
            joint_idx (int): Index of the joint.
            angle (float): Calculated angle for the joint.

        Returns:
            float: Angle adjusted by the motor offset.
        """
        joint_type = joint_idx % 3  # 0: abduction, 1: rotation, 2: elbow
        if joint_type == 0:
            return angle + self.motor_offsets['abduction']
        elif joint_type == 1:
            return angle + self.motor_offsets['rotation']
        else:  # elbow joint
            return angle + self.motor_offsets['elbow']

    def avoid_obstacle(self, obstacles):
        """
        Determine appropriate avoidance behavior based on obstacle detection
        Returns: True if avoidance maneuver was initiated or is ongoing
        """
        current_time = self.robot.getTime()

        # Don't start new avoidance during cooldown period
        if (not self.avoiding_obstacle and
                current_time - self.last_avoidance_end_time < self.post_avoidance_cooldown):
            return False

        # Check if we should continue existing avoidance maneuver
        if self.avoiding_obstacle:
            time_avoiding = current_time - self.avoidance_start_time

            # If we've been avoiding for too long, try a different direction
            if time_avoiding > self.max_avoidance_time:
                self.logger.info("Switching avoidance direction after timeout")
                self.avoidance_direction = (RotationDirection.RIGHT
                                            if self.avoidance_direction == RotationDirection.LEFT
                                            else RotationDirection.LEFT)
                self.avoidance_start_time = current_time
                self.forward_recovery_attempts = 0
                self.set_rotation(self.avoidance_direction,
                                  turn_rate_multiplier=1.5)
                return True

            # After minimum turning time, try moving forward if path is clear
            if time_avoiding > self.min_avoidance_time:
                # Only attempt forward movement if center is clear and we haven't exceeded attempts
                if not obstacles['center'] and self.forward_recovery_attempts < self.max_recovery_attempts:
                    self.set_gait(self.forward_recovery_time,
                                  RotationDirection.NONE, 0.5)
                    self.forward_recovery_attempts += 1
                    self.logger.info(
                        f"Attempting forward recovery ({self.forward_recovery_attempts}/{self.max_recovery_attempts})")
                else:
                    # Continue turning if sides are blocked
                    self.set_rotation(
                        self.avoidance_direction, turn_rate_multiplier=1.2)
            else:
                # Resume turning if forward path is blocked or too many attempts
                self.set_rotation(self.avoidance_direction,
                                  turn_rate_multiplier=1.2)
            return True

        # Start new avoidance maneuver if needed
        if obstacles['center'] or obstacles['left'] or obstacles['right']:
            self.avoiding_obstacle = True
            self.avoidance_start_time = current_time
            self.forward_recovery_attempts = 0

            # Determine best avoidance direction
            if obstacles['center']:
                if obstacles['left'] and not obstacles['right']:
                    self.avoidance_direction = RotationDirection.RIGHT
                elif obstacles['right'] and not obstacles['left']:
                    self.avoidance_direction = RotationDirection.LEFT
                else:
                    # When both sides are blocked or clear, maintain previous direction if it exists
                    self.avoidance_direction = (RotationDirection.RIGHT
                                                if not hasattr(self, 'last_avoidance_direction')
                                                else self.last_avoidance_direction)
            elif obstacles['left']:
                self.avoidance_direction = RotationDirection.RIGHT
            else:  # obstacle on right
                self.avoidance_direction = RotationDirection.LEFT

            self.set_rotation(self.avoidance_direction,
                              turn_rate_multiplier=1.2)
            self.logger.info(
                f"Starting avoidance maneuver: {self.avoidance_direction}")
            return True

        return False

    def resume_normal_movement(self):
        """Reset obstacle avoidance state and resume normal movement"""
        if self.avoiding_obstacle:
            self.avoiding_obstacle = False
            self.last_avoidance_direction = self.avoidance_direction
            self.avoidance_direction = None
            self.last_avoidance_end_time = self.robot.getTime()
            self.set_gait(5.0, RotationDirection.NONE)
            self.logger.info("Resuming normal movement")

    def update(self):
        """
        Update motor positions every simulation step.

        This method computes the current joint angles based on the selected gait,
        applies motor offsets, and commands the motors. It also advances the simulation
        using the provided step callback.
        """
        if self.gait_mode == "stand":
            self.set_standing_positions()
        else:
            # Generate new foot trajectories using the BezierGait generator
            # Simplified assumption: all feet in contact
            contacts = [1, 1, 1, 1]

            # Generate new foot trajectories
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

            # Apply joint angles to motors
            motor_positions = []
            for leg in joint_angles:
                motor_positions.extend(leg)

            # Command the motors
            for i, motor in enumerate(self.motors):
                motor.setPosition(
                    self.apply_motor_offset(i, motor_positions[i]))

        # Advance the simulation by one step.
        self.step()

    def set_standing_positions(self):
        """
        Command all joints to their standing posture positions.

        This method sends position commands to each motor based on a default
        standing configuration and applies the necessary offsets.
        """
        # Base positions for a neutral, stable standing posture.
        base_positions = [
            0.0, 0.0, 0.0,  # front left leg
            0.0, 0.0, 0.0,  # front right leg
            0.0, 0.0, 0.0,  # rear left leg
            0.0, 0.0, 0.0   # rear right leg
        ]

        # Command each motor with its adjusted standing position.
        for i, pos in enumerate(base_positions):
            self.motors[i].setPosition(self.apply_motor_offset(i, pos))
