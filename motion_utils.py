# utils/motion_utils.py

def movement_decomposition(motors, target_positions, duration, time_step):
    """
    Gradually move all joints from their current positions to the target positions
    over the specified duration.
    
    motors: list of motor device objects.
    target_positions: list of target joint positions.
    duration: time in seconds.
    time_step: simulation time step in milliseconds.
    """
    n_steps = int(duration * 1000 / time_step)
    current_positions = [motor.getTargetPosition() for motor in motors]
    increments = [(target - current) / n_steps for target,
                  current in zip(target_positions, current_positions)]

    for _ in range(n_steps):
        for i, motor in enumerate(motors):
            current_positions[i] += increments[i]
            motor.setPosition(current_positions[i])
        # The calling function should perform a simulation step here.
