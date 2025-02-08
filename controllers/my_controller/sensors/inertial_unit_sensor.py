# sensors/inertial_unit_sensor.py

class InertialUnitSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger
        self.inertial_unit = self.robot.getDevice(self.device_name)
        if self.inertial_unit:
            self.inertial_unit.enable(self.time_step)
            if self.logger:
                self.logger.info("Inertial Unit sensor enabled.")
        else:
            if self.logger:
                self.logger.warning("Inertial Unit sensor not found.")

    def get_heading(self):
        if self.inertial_unit:
            # Assuming getRollPitchYaw() returns [roll, pitch, yaw] in radians.
            return self.inertial_unit.getRollPitchYaw()[2]
        return 0.0
