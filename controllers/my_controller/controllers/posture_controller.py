# controllers/posture_controller.py

from utils.config import POSTURES
from utils.motion_utils import movement_decomposition


class PostureController:
    def __init__(self, motors, time_step, step_callback):
        """
        motors: list of motor devices.
        time_step: simulation time step (ms).
        step_callback: function to advance the simulation (e.g., a wrapper around robot.step).
        """
        self.motors = motors
        self.time_step = time_step
        self.step = step_callback

    def apply_posture(self, posture_name, duration):
        """Smoothly transition the robot into the desired posture."""
        if posture_name not in POSTURES:
            raise ValueError(f"Posture '{posture_name}' is not defined.")
        target_positions = POSTURES[posture_name]
        n_steps = int(duration * 1000 / self.time_step)
        current_positions = [motor.getTargetPosition()
                             for motor in self.motors]
        increments = [(target - current) / n_steps for target,
                      current in zip(target_positions, current_positions)]
        for _ in range(n_steps):
            for i, motor in enumerate(self.motors):
                current_positions[i] += increments[i]
                motor.setPosition(current_positions[i])
            self.step()

    def stand_up(self, duration=4.0):
        self.apply_posture("stand_up", duration)

    def lie_down(self, duration=4.0):
        self.apply_posture("lie_down", duration)

    def sit_down(self, duration=4.0):
        self.apply_posture("sit_down", duration)

    def give_paw(self):
        # A placeholder for a complex sequence.
        self.apply_posture("sit_down", 4.0)
        # (Insert dynamic motor commands for the "paw" movement here.)
        self.apply_posture("sit_down", 4.0)
