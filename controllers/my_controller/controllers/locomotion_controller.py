# controllers/locomotion_controller.py
import math


class LocomotionController:
    def __init__(self, robot, motors, time_step, step_callback, get_heading_callback=None):
        self.robot = robot
        self.motors = motors
        self.time_step = time_step
        self.step = step_callback
        self.get_heading = get_heading_callback

        # Immediately set motors to a stable standing posture.
        self.set_standing_positions()

        # Gait state variables.
        # gait_mode can be "walk", "rotate", or "stand".
        self.gait_mode = "stand"
        self.gait_start_time = self.robot.getTime()
        # until when the current command is valid
        self.gait_deadline = self.robot.getTime()
        self.gait_params = {}

    def set_gait(self, mode, duration, params=None):
        """
        Issue a non-blocking gait command.

        mode: "walk" or "rotate"
        duration: how long (in seconds) the command should be valid
        params: dictionary for extra parameters (e.g. rotation direction)
        """
        if params is None:
            params = {}
        current_time = self.robot.getTime()
        # Only reset the gait start time if the mode is changing.
        if self.gait_mode != mode:
            self.gait_mode = mode
            self.gait_start_time = current_time
        # In any case, update the deadline so the command remains valid.
        self.gait_deadline = current_time + duration
        self.gait_params = params

    def update(self):
        """Call this method every simulation step to update motor positions."""
        current_time = self.robot.getTime()
        # If the command has expired, revert to standing.
        if current_time > self.gait_deadline:
            self.gait_mode = "stand"
            self.set_standing_positions()
            return

        elapsed = current_time - self.gait_start_time
        if self.gait_mode == "walk":
            self.update_walk(elapsed)
        elif self.gait_mode == "rotate":
            self.update_rotate(elapsed)
        else:
            self.set_standing_positions()

    def update_walk(self, elapsed):
        # Walking gait parameters
        gait_frequency = 0.9  # Reduced frequency for more stability
        omega = 2 * math.pi * gait_frequency
        phase = omega * elapsed

        # Reduce shoulder amplitude for better balance
        shoulder_amplitude = 0.2  # Further reduced for stability

        # Phase offsets for each leg (trot gait)
        pair1_phase = phase
        pair2_phase = phase + math.pi

        def compute_leg_lift(phase_value):
            # Using the ranges from the C example
            stance_angle = 0.0     # Neutral position when standing
            swing_angle = -0.2     # Small lift during swing

            # Smoother transitions between stance and swing
            lift = max(0, math.sin(phase_value))
            lift = math.pow(lift, 1.0)  # Slightly sharper transitions

            return stance_angle + (swing_angle - stance_angle) * lift

        # Compute leg lifts
        fl_elbow = compute_leg_lift(pair1_phase)
        rr_elbow = compute_leg_lift(pair1_phase)
        fr_elbow = compute_leg_lift(pair2_phase)
        rl_elbow = compute_leg_lift(pair2_phase)

        # Smaller forward bias based on C example
        forward_bias = -0.02

        # Compute shoulder rotations with forward bias
        fl_shoulder = forward_bias - shoulder_amplitude * math.sin(pair1_phase)
        rr_shoulder = forward_bias + shoulder_amplitude * math.sin(pair1_phase)
        fr_shoulder = forward_bias - shoulder_amplitude * math.sin(pair2_phase)
        rl_shoulder = forward_bias + shoulder_amplitude * math.sin(pair2_phase)

        # Set motor positions
        # Front Left leg
        self.motors[1].setPosition(fl_shoulder)
        self.motors[2].setPosition(fl_elbow)

        # Front Right leg
        self.motors[4].setPosition(fr_shoulder)
        self.motors[5].setPosition(fr_elbow)

        # Rear Left leg
        self.motors[7].setPosition(rl_shoulder)
        self.motors[8].setPosition(rl_elbow)

        # Rear Right leg
        self.motors[10].setPosition(rr_shoulder)
        self.motors[11].setPosition(rr_elbow)

        # Abduction angles from C example
        self.motors[0].setPosition(-0.1)  # FL abduction
        self.motors[3].setPosition(0.1)   # FR abduction
        self.motors[6].setPosition(-0.1)  # RL abduction
        self.motors[9].setPosition(0.1)   # RR abduction

    def update_rotate(self, elapsed):
        # Rotation gait parameters.
        frequency = 0.5  # Hz (adjust if needed)
        omega = 2 * math.pi * frequency
        phase = omega * elapsed
        # +1 for rotate right, -1 for left
        direction = self.gait_params.get("direction", 1)

        # Increase the amplitude for a more pronounced rotation.
        shoulder_amplitude = 0.25  # Increased amplitude for shoulder rotation.
        elbow_amplitude = 0.15     # Increased amplitude for elbow movement.

        # To generate a net turning moment, we bias the gait:
        # Left legs: swing forward (or less retracted) relative to right legs.
        # Right legs: swing back (or more retracted) relative to left legs.
        #
        # Compute a baseline oscillation.
        oscillation = math.sin(phase)
        # Introduce a bias offset for left and right sides.
        left_bias = 0.15 * direction  # Positive bias when rotating right.
        right_bias = -0.15 * direction

        # For left legs (FL and RL):
        left_rotation = left_bias + shoulder_amplitude * oscillation
        # Let the elbow (knee) extend more when the leg is in swing phase.
        left_elbow = -0.4 + elbow_amplitude * (0.5 + 0.5 * oscillation)

        # For right legs (FR and RR):
        right_rotation = right_bias - shoulder_amplitude * oscillation
        right_elbow = -0.4 + elbow_amplitude * (0.5 - 0.5 * oscillation)

        # Set positions for left legs.
        self.motors[1].setPosition(left_rotation)  # FL shoulder rotation
        self.motors[2].setPosition(left_elbow)       # FL elbow
        self.motors[7].setPosition(left_rotation)    # RL shoulder rotation
        self.motors[8].setPosition(left_elbow)       # RL elbow

        # Set positions for right legs.
        self.motors[4].setPosition(right_rotation)   # FR shoulder rotation
        self.motors[5].setPosition(right_elbow)        # FR elbow
        self.motors[10].setPosition(right_rotation)    # RR shoulder rotation
        self.motors[11].setPosition(right_elbow)       # RR elbow

        # Keep the abduction joints fixed.
        self.motors[0].setPosition(-0.1)  # FL abduction
        self.motors[3].setPosition(0.1)   # FR abduction
        self.motors[6].setPosition(-0.1)  # RL abduction
        self.motors[9].setPosition(0.1)   # RR abduction

    def set_standing_positions(self):
        """Place all joints in a stable standing posture."""
        self.motors[0].setPosition(-0.1)  # FL abduction
        self.motors[1].setPosition(0.0)   # FL rotation
        self.motors[2].setPosition(0.0)   # FL elbow
        self.motors[3].setPosition(0.1)   # FR abduction
        self.motors[4].setPosition(0.0)   # FR rotation
        self.motors[5].setPosition(0.0)   # FR elbow
        self.motors[6].setPosition(-0.1)  # RL abduction
        self.motors[7].setPosition(0.0)   # RL rotation
        self.motors[8].setPosition(0.0)   # RL elbow
        self.motors[9].setPosition(0.1)   # RR abduction
        self.motors[10].setPosition(0.0)  # RR rotation
        self.motors[11].setPosition(0.0)  # RR elbow
