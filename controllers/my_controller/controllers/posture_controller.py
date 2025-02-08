# controllers/posture_controller.py
from utils.config import POSTURES
from utils.motion_utils import movement_decomposition


class PostureController:
    def __init__(self, motors, time_step, step_callback):
        self.motors = motors
        self.time_step = time_step
        self.step = step_callback

    def apply_posture(self, posture_name, duration):
        if posture_name not in POSTURES:
            raise ValueError(f"Posture '{posture_name}' is not defined.")
        target_positions = POSTURES[posture_name]
        movement_decomposition(self.motors, target_positions,
                               duration, self.time_step, self.step)

    def stand_up(self, duration=4.0):
        self.apply_posture("stand_up", duration)

    def lie_down(self, duration=4.0):
        self.apply_posture("lie_down", duration)

    def sit_down(self, duration=4.0):
        self.apply_posture("sit_down", duration)

    def give_paw(self):
        self.apply_posture("sit_down", 4.0)
        # Insert additional motor commands for "paw" movement here.
        self.apply_posture("sit_down", 4.0)
