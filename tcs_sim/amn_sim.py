# amn_sim.py
import time
import threading

from subsystem_base import SubsystemBase
from bus_config import log


class AMNSubsystem(SubsystemBase):
    """
    Simulated Auxiliary Mechanism Node (AMN)
    Controls: FOCUS, DFOCUS, MIRROR_COVER, MOVE_FOLD
    """

    def __init__(self):
        super().__init__("AMN")

        self.focus_pos_mm = 0.0
        self.mirror_cover = "CLOSE"
        self.fold_position = "STOW"

        self.vel_limit_mm_s = 0.6  # mm/sec for focus
        self.sim_speed = 1.0

        self.command_table = {
            "FOCUS":        self._cmd_FOCUS,
            "DFOCUS":       self._cmd_DFOCUS,
            "MIRROR_COVER": self._cmd_MIRROR_COVER,
            "MOVE_FOLD":    self._cmd_MOVE_FOLD,
        }

    def _simulate_motion(self, target_val: float, current_attr: str, vel_attr: str, timeout_s: float):
        start_val = getattr(self, current_attr)
        delta = target_val - start_val
        distance = abs(delta)
        velocity = getattr(self, vel_attr)

        if velocity <= 0:
            return {"status": "ERROR", "error": "MECHERR"}

        t_required_real = distance / velocity if distance > 0.0 else 0.0
        t_required_sim = t_required_real / max(self.sim_speed, 1e-6)

        direction = 1.0 if delta >= 0 else -1.0

        def _motion_loop():
            t0 = time.time()
            last_t = t0

            while True:
                now = time.time()
                elapsed = now - t0

                if elapsed >= t_required_sim:
                    setattr(self, current_attr, target_val)
                    return

                dt = now - last_t
                last_t = now

                step = direction * velocity * self.sim_speed * dt
                current_val = getattr(self, current_attr) + step

                # Clamp
                if (direction > 0 and current_val > target_val) or (direction < 0 and current_val < target_val):
                    current_val = target_val

                setattr(self, current_attr, current_val)
                time.sleep(0.1)

        thread = threading.Thread(target=_motion_loop)
        thread.start()
        thread.join(timeout_s)

        if thread.is_alive():
            return {"status": "ERROR", "error": "TIMEDOUT"}

        return {"status": "OK", "error": "OK"}

    def _cmd_FOCUS(self, cmd_id, params):
        pos = params.get("position")
        if pos is None:
            self.send_result("ERROR", cmd_id, error_code="BADPARAM")
            return

        try:
            target = float(pos)
        except Exception:
            self.send_result("ERROR", cmd_id, error_code="BADPARAM")
            return

        self.send_result("ACK", cmd_id)
        res = self._simulate_motion(target, "focus_pos_mm", "vel_limit_mm_s", timeout_s=40.0)

        if res["status"] == "ERROR":
            self.send_result("ERROR", cmd_id, error_code=res["error"])
            return

        self.send_result("DONE", cmd_id)

    def _cmd_DFOCUS(self, cmd_id, params):
        delta = params.get("offset")
        if delta is None:
            self.send_result("ERROR", cmd_id, error_code="BADPARAM")
            return

        try:
            target = self.focus_pos_mm + float(delta)
        except Exception:
            self.send_result("ERROR", cmd_id, error_code="BADPARAM")
            return

        self.send_result("ACK", cmd_id)
        res = self._simulate_motion(target, "focus_pos_mm", "vel_limit_mm_s", timeout_s=40.0)

        if res["status"] == "ERROR":
            self.send_result("ERROR", cmd_id, error_code=res["error"])
            return

        self.send_result("DONE", cmd_id)

    def _cmd_MIRROR_COVER(self, cmd_id, params):
        state = (params.get("state") or "CLOSE").upper()
        if state not in ("OPEN", "CLOSE"):
            self.send_result("ERROR", cmd_id, error_code="BADPARAM")
            return

        self.mirror_cover = state
        self.send_result("ACK", cmd_id)
        time.sleep(1)
        self.send_result("DONE", cmd_id)

    def _cmd_MOVE_FOLD(self, cmd_id, params):
        state = (params.get("state") or "STOW").upper()
        if state not in ("STOW", "1", "2", "3", "4"):
            self.send_result("ERROR", cmd_id, error_code="BADPARAM")
            return

        self.fold_position = state
        self.send_result("ACK", cmd_id)
        time.sleep(1)
        self.send_result("DONE", cmd_id)

    def handle_command(self, msg):
        cmd_id = msg.get("cmd_id")
        command = (msg.get("command") or "").upper()
        params = msg.get("params", {}) or {}

        handler = self.command_table.get(command)
        if not handler:
            log("AMN", f"Unknown command '{command}'")
            self.send_result("ERROR", cmd_id, error_code="NETUNK")
            return

        handler(cmd_id, params)
