# cas_sim.py — FINAL corrected version for TCS core

import threading
import time
from subsystem_base import SubsystemBase
from bus_config import log


class CASSubsystem(SubsystemBase):
    """
    Simulated Cassegrain Rotator
    TCS sends:
        CAS  <deg>
    JSON form:
        {"command":"CAS","params":{"deg":X}}
    """

    def __init__(self):
        super().__init__("CAS")

        self.current_pos_deg = 0.0
        self.target_pos_deg = 0.0
        self.current_pos = self.current_pos_deg

        self.state = "STOPPED"
        self.last_error = "OK"

        self.vel_limit_deg_s = 2.0
        self.sim_speed = 1.0

        self._motion_thread = None
        self._motion_stop = False

        self.command_table = {
            "CAS": self._cmd_CAS,
        }

    # ----------------------------------------------------
    def _set_error(self, code):
        self.last_error = code
        self.state = "ERROR"

    def _reject_if_blocked(self):
        if self.state in ("OFF-LINE", "OVERRIDE", "UNKNOWN"):
            self._set_error("STATEREJECT")
            return "STATEREJECT"
        return None

    # ----------------------------------------------------
    def _start_move(self, target_deg, timeout_s):

        if self._motion_thread and self._motion_thread.is_alive():
            self._motion_stop = True
            self._set_error("OVER")

        start = self.current_pos_deg
        delta = target_deg - start
        dist = abs(delta)

        if dist == 0:
            self.current_pos_deg = target_deg
            self.current_pos = target_deg
            self.state = "IN POSN"
            self.last_error = "OK"
            self.send_status()
            return {"status": "OK", "error": "OK"}

        t_real = dist / self.vel_limit_deg_s
        if t_real > timeout_s:
            self._set_error("TIMEDOUT")
            return {"status": "ERROR", "error": "TIMEDOUT"}

        t_sim = t_real / max(self.sim_speed, 1e-6)
        self.target_pos_deg = target_deg
        direction = 1 if delta >= 0 else -1
        self._motion_stop = False

        def _loop():
            self.state = "MOVING"
            t0 = time.time()
            last = t0

            self.send_status()

            while True:
                if self._motion_stop or not self.running:
                    self.state = "STOPPED"
                    self.send_status()
                    return

                now = time.time()
                if now - t0 >= t_sim:
                    self.current_pos_deg = target_deg
                    self.current_pos = target_deg
                    self.state = "IN POSN"
                    self.last_error = "OK"
                    self.send_status()
                    return

                dt = now - last
                last = now

                step = direction * self.vel_limit_deg_s * self.sim_speed * dt
                self.current_pos_deg += step
                self.current_pos = self.current_pos_deg

                if direction > 0 and self.current_pos_deg > target_deg:
                    self.current_pos_deg = target_deg
                if direction < 0 and self.current_pos_deg < target_deg:
                    self.current_pos_deg = target_deg

                self.send_status()
                time.sleep(0.10)

        self._motion_thread = threading.Thread(target=_loop, daemon=True)
        self._motion_thread.start()
        return {"status": "OK", "error": "OK"}

    # ----------------------------------------------------
    def _cmd_CAS(self, cmd_id, params):

        self.send_result("ACK", cmd_id=cmd_id)

        raw = params.get("deg")
        if raw is None:
            self._set_error("BADPARAM")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADPARAM")
            return

        try:
            angle = float(raw)
        except:
            self._set_error("BADPARAM")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADPARAM")
            return

        # no special range for CAS
        block = self._reject_if_blocked()
        if block:
            self.send_result("ERROR", cmd_id=cmd_id, error_code=block)
            return

        res = self._start_move(angle, timeout_s=120)
        if res["status"] == "ERROR":
            self.send_result("ERROR", cmd_id=cmd_id, error_code=res["error"])
            return

        log("CAS", f"Accepted CAS {angle:.3f}")

    # ----------------------------------------------------
    def handle_command(self, msg):
        cmd_id = msg.get("cmd_id")
        cmd = (msg.get("command") or "").upper()
        params = msg.get("params", {}) or {}

        f = self.command_table.get(cmd)
        if not f:
            self.send_result("ERROR", cmd_id=cmd_id, error_code="NETUNK")
            return

        f(cmd_id, params)
