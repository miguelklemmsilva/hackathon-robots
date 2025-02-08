import math


class LocomotionController:
    def __init__(self, robot, motors, time_step, step_callback):
        """
        robot: The Webots Robot instance.
        motors: List of motor devices.
        time_step: Simulation time step in milliseconds.
        step_callback: Function to advance the simulation (e.g., a wrapper around robot.step).
        """
        self.robot = robot
        self.motors = motors
        self.time_step = time_step
        self.step = step_callback

        # Base positions for a standing posture.
        # Order: front left, front right, rear left, rear right legs (each with 3 joints)
        # (These should match the "stand_up" posture.)
        self.base_positions = [-0.1, 0.0, 0.0,  # front left leg: abduction, rotation, elbow
                               0.1,  0.0, 0.0,  # front right leg
                               -0.1,  0.0, 0.0,  # rear left leg
                               0.1,  0.0, 0.0]  # rear right leg

    def walk_forward(self, duration):
        """
        Simulate forward walking by executing a simple trot gait for the given duration.
        This method modulates the shoulder rotation and elbow joints using a sinusoidal pattern.
        """
        start_time = self.robot.getTime()
        # Gait parameters
        gait_frequency = 0.5  # cycles per second (Hz)
        omega = 2 * math.pi * gait_frequency
        amplitude_rot = 0.1    # radians offset for shoulder rotation joints
        amplitude_elbow = 0.05  # radians offset for elbow joints

        # Leg groupings (by indices):
        #   Group 1 (phase = sin(omega*t)): front left leg and rear right leg
        #     - Front left leg: motor indices 1 (shoulder rotation) and 2 (elbow)
        #     - Rear right leg: motor indices 10 (shoulder rotation) and 11 (elbow)
        #   Group 2 (phase shifted by pi): front right leg and rear left leg
        #     - Front right leg: motor indices 4 (shoulder rotation) and 5 (elbow)
        #     - Rear left leg: motor indices 7 (shoulder rotation) and 8 (elbow)
        while self.robot.getTime() - start_time < duration:
            t = self.robot.getTime() - start_time

            # Compute phase offsets for the two groups.
            offset_group1 = math.sin(omega * t)
            offset_group2 = math.sin(omega * t + math.pi)

            # Update Group 1 joints:
            # Front left leg:
            self.motors[1].setPosition(
                self.base_positions[1] + amplitude_rot * offset_group1)
            self.motors[2].setPosition(
                self.base_positions[2] + amplitude_elbow * offset_group1)
            # Rear right leg:
            self.motors[10].setPosition(
                self.base_positions[10] + amplitude_rot * offset_group1)
            self.motors[11].setPosition(
                self.base_positions[11] + amplitude_elbow * offset_group1)

            # Update Group 2 joints:
            # Front right leg:
            self.motors[4].setPosition(
                self.base_positions[4] + amplitude_rot * offset_group2)
            self.motors[5].setPosition(
                self.base_positions[5] + amplitude_elbow * offset_group2)
            # Rear left leg:
            self.motors[7].setPosition(
                self.base_positions[7] + amplitude_rot * offset_group2)
            self.motors[8].setPosition(
                self.base_positions[8] + amplitude_elbow * offset_group2)

            # Keep the abduction joints (indices 0, 3, 6, 9) at their base positions.
            self.motors[0].setPosition(self.base_positions[0])
            self.motors[3].setPosition(self.base_positions[3])
            self.motors[6].setPosition(self.base_positions[6])
            self.motors[9].setPosition(self.base_positions[9])

            self.step()

    def rotate_left(self, duration):
        """
        Simulate a simple rotation to the left.
        This is a placeholder—rotate by adjusting the leg motions asymmetrically.
        """
        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < duration:
            # For turning, you might adjust one side's gait more than the other.
            # Here we simply call step() as a placeholder.
            self.step()
