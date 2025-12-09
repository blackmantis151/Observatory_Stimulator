# az_sim.py — FINAL corrected version for TCS core

import threading
import time
from subsystem_base import SubsystemBase
from bus_config import log


class AZSubsystem(SubsystemBase):
    """
    Simulated AZ axis
    Commands supported:
        AZIMUTH  angle
        UNWRAP_AZ
    Valid range: -180 to +360
    """

    def __init__(self):
        super().__init__("AZ")

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
            "AZIMUTH": self._cmd_AZIMUTH,
            "UNWRAP_AZ": self._cmd_UNWRAP_AZ,
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
    # Motion engine (same pattern as ALT)
    # ----------------------------------------------------
    def _start_move(self, target_deg, timeout_s):

        if self._motion_thread and self._motion_thread.is_alive():
            self._motion_stop = True
            self._set_error("OVER")

        start = self.current_pos_deg
        delta = target_deg - start
        dist = abs(delta)

        if dist == 0.0:
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
        direction = 1.0 if delta >= 0 else -1.0
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
    def _cmd_AZIMUTH(self, cmd_id, params):

        self.send_result("ACK", cmd_id=cmd_id)

        angle_raw = params.get("angle")
        if angle_raw is None:
            self._set_error("BADPARAM")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADPARAM")
            return

        try:
            angle = float(angle_raw)
        except:
            self._set_error("BADPARAM")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADPARAM")
            return

        if angle < -180 or angle > 360:
            self._set_error("BADRANGE")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADRANGE")
            return

        block = self._reject_if_blocked()
        if block:
            self.send_result("ERROR", cmd_id=cmd_id, error_code=block)
            return

        res = self._start_move(angle, timeout_s=300)
        if res["status"] == "ERROR":
            self.send_result("ERROR", cmd_id=cmd_id, error_code=res["error"])
            return

        log("AZ", f"Accepted AZIMUTH {angle:.3f}")

    # ----------------------------------------------------
    def _cmd_UNWRAP_AZ(self, cmd_id, params):
        self.send_result("ACK", cmd_id=cmd_id)

        x = self.current_pos_deg
        min_lim, max_lim = -180, 360

        forward_ok = x + 360 <= max_lim
        backward_ok = x - 360 >= min_lim

        if not (forward_ok or backward_ok):
            self._set_error("BADRANGE")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADRANGE")
            return

        target = x + 360 if forward_ok else x - 360

        res = self._start_move(target, timeout_s=220)
        if res["status"] == "ERROR":
            self.send_result("ERROR", cmd_id=cmd_id, error_code=res["error"])
            return

        log("AZ", f"UNWRAP → {target:.3f}")

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
