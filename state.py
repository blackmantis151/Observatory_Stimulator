import time

class SimulatorState:
    """Holds current motor angles and clock."""

    def __init__(self, sim_cfg):
        self.alt = 0.0
        self.az = 0.0
        self.cas = 0.0
        self.speed = sim_cfg['motor_speed_deg_per_s']
        self.start_time = time.time()

    def update_towards(self, target_alt, target_az, target_cas, dt):
        """Move each axis towards target by speed*dt."""
        def step(cur, tgt):
            delta = tgt - cur
            max_move = self.speed * dt
            if abs(delta) <= max_move:
                return tgt
            return cur + max_move * (1 if delta > 0 else -1)

        self.alt = step(self.alt, target_alt)
        self.az  = step(self.az, target_az)
        self.cas = step(self.cas, target_cas)

    def get_feedback(self):
        """Return current angles."""
        return {'alt': self.alt, 'az': self.az, 'cas': self.cas}
