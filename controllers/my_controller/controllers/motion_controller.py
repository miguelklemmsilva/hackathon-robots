# controllers/locomotion_controller.py

class LocomotionController:
    def __init__(self, robot, time_step, step_callback):
        """
        robot: The Webots Robot instance.
        time_step: Simulation time step in ms.
        step_callback: Function to advance the simulation.
        """
        self.robot = robot
        self.time_step = time_step
        self.step = step_callback

    def walk_forward(self, duration):
        """
        Simulate walking forward for the specified duration (seconds).
        Replace this with your gait planning algorithm as needed.
        """
        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < duration:
            # Insert advanced gait control logic here.
            self.step()

    def rotate_left(self, duration):
        """
        Simulate rotating left for the specified duration (seconds).
        Replace this with joint-level rotation control if desired.
        """
        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < duration:
            # Insert rotation control logic here.
            self.step()