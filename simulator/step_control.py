class StepController:
    def __init__(self):
        self.x_step = 0
        self.rotation_step = 6
        self.grip_closed = False
        self.x_max = 5
        self.rotation_max = 12
        self.rotation_deg_per_step = 15

    def snapshot(self):
        return {
            "x_step": self.x_step,
            "rotation_step": self.rotation_step,
            "rotation_deg": self.rotation_step * self.rotation_deg_per_step,
            "grip_closed": self.grip_closed,
            "simulation_only": True
        }

    def set_x(self, target):
        if type(target) is not int or not 0 <= target <= self.x_max:
            raise ValueError("X step out of range")
        self.x_step = target
        return self.snapshot()

    def set_rotation(self, target):
        if type(target) is not int or not 0 <= target <= self.rotation_max:
            raise ValueError("Rotation step out of range")
        self.rotation_step = target
        return self.snapshot()

    def set_grip(self, closed):
        if type(closed) is not bool:
            raise ValueError("grip_closed must be boolean")
        self.grip_closed = closed
        return self.snapshot()
