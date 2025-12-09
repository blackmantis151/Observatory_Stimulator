# alt_sim.py — corrected and synchronised with TCS core

import threading
import time

from subsystem_base import SubsystemBase
from bus_config import log


class ALTSubsystem(SubsystemBase):
    """
    Simulated ALT axis (mechanism control).

    Implements the TTL-like command:
        ALTITUDE angle

    - Valid range: 20.0 to 91.0 degrees
    - Timeout: 100 s
    - States: STOPPED, MOVING, IN POSN, LIMIT, ERROR
    - Motion simulated with constant velocity (no acceleration).
    """

    def __init__(self):
        super().__init__("ALT")

        # --- canonical position variables expected by SubsystemBase/SCC ---
        self.current_pos_deg = 45.0       # initial altitude
        self.target_pos_deg = 45.0

        # Legacy alias if anything still uses current_pos
        self.current_pos = self.current_pos_deg

        self.state = "STOPPED"
        self.last_error = "OK"

        # --- motion model parameters ---
        self.vel_limit_deg_s = 2.0        # physical velocity (deg/sec)
        self.sim_speed = 1.0              # 1.0 = real time, <1 slower, >1 faster

        self._motion_thread = None
        self._motion_stop = False

        # --- command table (TCS sends ALTITUDE, STOP ALT) ---
        self.command_table = {
            "ALTITUDE": self._cmd_ALTITUDE,
            "STOP":     self._cmd_STOP,    # STOP ALT
        }

    # ------------------------------------------------------------
    # Helper: set error condition
    # ------------------------------------------------------------
    def _set_error(self, code: str):
        self.last_error = code
        self.state = "ERROR"

    # ------------------------------------------------------------
    # Helper: reject when mechanism in OFF-LINE / OVERRIDE / UNKNOWN
    # ------------------------------------------------------------
    def _reject_if_blocked(self):
        if self.state in ("OFF-LINE", "OVERRIDE", "UNKNOWN"):
            self._set_error("STATEREJECT")
            return "STATEREJECT"
        return None

    # ------------------------------------------------------------
    # Motion engine
    # ------------------------------------------------------------
    def _start_move(self, target_deg: float, timeout_s: float):
        """
        Start motion toward target_deg at vel_limit_deg_s.
        Uses current_pos_deg / target_pos_deg as canonical variables.
        """

        # If another motion is running → override
        if self._motion_thread and self._motion_thread.is_alive():
            self._motion_stop = True
            self._set_error("OVER")

        start = self.current_pos_deg
        delta = target_deg - start
        distance = abs(delta)

        if self.vel_limit_deg_s <= 0.0:
            self._set_error("MECHERR")
            return {"status": "ERROR", "error": "MECHERR"}

        if distance > 0.0:
            t_required_real = distance / self.vel_limit_deg_s
        else:
            t_required_real = 0.0

        if t_required_real > timeout_s:
            self._set_error("TIMEDOUT")
            return {"status": "ERROR", "error": "TIMEDOUT"}

        # apply simulation speed scaling
        self.sim_speed = max(self.sim_speed, 1e-6)
        t_required_sim = t_required_real / self.sim_speed

        # set demand
        self.target_pos_deg = target_deg

        # no movement needed
        if t_required_sim <= 0.0 or distance == 0.0:
            self.current_pos_deg = target_deg
            self.current_pos = self.current_pos_deg
            self.state = "IN POSN"
            self.last_error = "OK"
            self.send_status()
            return {"status": "OK", "error": "OK"}

        direction = 1.0 if delta >= 0 else -1.0
        self._motion_stop = False

        def _motion_loop():
            self.state = "MOVING"
            t0 = time.time()
            last_t = t0

            # initial status at start of motion
            self.send_status()

            while True:
                if self._motion_stop or not self.running:
                    # STOP or shutdown
                    self.state = "STOPPED"
                    self.send_status()
                    return

                now = time.time()
                elapsed = now - t0

                if elapsed >= t_required_sim:
                    # snap to target
                    self.current_pos_deg = target_deg
                    self.current_pos = self.current_pos_deg
                    self.state = "IN POSN"
                    self.last_error = "OK"
                    self.send_status()
                    return

                dt = now - last_t
                last_t = now

                step = direction * self.vel_limit_deg_s * self.sim_speed * dt
                self.current_pos_deg += step
                self.current_pos = self.current_pos_deg

                # clamp
                if direction > 0 and self.current_pos_deg > target_deg:
                    self.current_pos_deg = target_deg
                    self.current_pos = self.current_pos_deg
                if direction < 0 and self.current_pos_deg < target_deg:
                    self.current_pos_deg = target_deg
                    self.current_pos = self.current_pos_deg

                # status update each step
                self.send_status()
                time.sleep(0.10)

        self._motion_thread = threading.Thread(target=_motion_loop, daemon=True)
        self._motion_thread.start()

        return {"status": "OK", "error": "OK"}

    # ------------------------------------------------------------
    # Command handler: ALTITUDE
    # ------------------------------------------------------------
    def _cmd_ALTITUDE(self, cmd_id, params):
        """
        Handler for command ALTITUDE.
        TCS sends:
            {"command": "ALTITUDE", "params": {"angle": X}}
        """

        # Immediate ACK
        self.send_result("ACK", cmd_id=cmd_id)

        angle_raw = params.get("angle")
        if angle_raw is None:
            self._set_error("BADPARAM")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADPARAM")
            return

        try:
            angle = float(angle_raw)
        except Exception:
            self._set_error("BADPARAM")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADPARAM")
            return

        # valid range
        if angle < 20.0 or angle > 91.0:
            self.state = "LIMIT"
            self._set_error("BADRANGE")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADRANGE")
            return

        # blocked?
        block = self._reject_if_blocked()
        if block:
            self.send_result("ERROR", cmd_id=cmd_id, error_code=block)
            return

        # begin motion
        res = self._start_move(angle, timeout_s=100.0)
        if res["status"] == "ERROR":
            self.send_result("ERROR", cmd_id=cmd_id, error_code=res["error"])
            return

        log("ALT", f"Accepted ALTITUDE move to {angle:.3f}°")

    # ------------------------------------------------------------
    # Command handler: STOP ALT
    # ------------------------------------------------------------
    def _cmd_STOP(self, cmd_id, params):
        """
        Handler for STOP on ALT mechanism.

        TCS sends on ALT bus:
            {"command": "STOP", "params": {"mechanism": "ALT"}}
        """

        mech = (params.get("mechanism") or "").upper()
        if mech not in ("", "ALT"):
            log("ALT", f"STOP received with unexpected mechanism '{mech}', treating as ALT")

        # signal motion loop to halt
        self._motion_stop = True

        # if no motion, just enforce STOPPED
        if not (self._motion_thread and self._motion_thread.is_alive()):
            self.state = "STOPPED"
            self.last_error = "OK"
            self.send_status()

        # ACK STOP
        self.send_result("ACK", cmd_id=cmd_id)

    # ------------------------------------------------------------
    # Dispatch from SubsystemBase
    # ------------------------------------------------------------
    def handle_command(self, msg: dict):
        cmd_id = msg.get("cmd_id")
        command = (msg.get("command") or "").upper()
        params = msg.get("params", {}) or {}

        handler = self.command_table.get(command)
        if not handler:
            log("ALT", f"Unknown command '{command}'")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="NETUNK")
            return

        handler(cmd_id, params)
