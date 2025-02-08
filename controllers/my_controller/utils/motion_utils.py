# utils/motion_utils.py

def movement_decomposition(motors, target_positions, duration, time_step, step_callback):
    """
    Gradually move all joints from their current positions to target_positions over duration seconds.
    """
    n_steps = int(duration * 1000 / time_step)
    current_positions = [motor.getTargetPosition() for motor in motors]
    increments = [(target - current) / n_steps for target,
                  current in zip(target_positions, current_positions)]

    for _ in range(n_steps):
        for i, motor in enumerate(motors):
            current_positions[i] += increments[i]
            motor.setPosition(current_positions[i])
        step_callback()
