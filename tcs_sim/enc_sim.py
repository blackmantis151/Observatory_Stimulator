# enc_sim.py
import threading
import time

from subsystem_base import SubsystemBase
from bus_config import log


class ENCSubsystem(SubsystemBase):
    """
    Simulated Enclosure (ENC) system
    Controls: ENCLOSURE, DOME

    - Dome Azimuth max speed: 2.0 deg/sec
    - Shutter movement time (open/close): assume 5 sec open, 9 sec close
    """

    def __init__(self):
        super().__init__("ENC")

        self.shutters = {"SHUTTER1": "CLOSE", "SHUTTER2": "CLOSE"}
        self.shutter_pos = {"SHUTTER1": 0.0, "SHUTTER2": 0.0}
        self.dome_az = 0.0

        self.shutter_motion_time = {"OPEN": 5.0, "CLOSE": 9.0}  # sec
        self.dome_az_speed = 2.0  # deg/sec

        self.command_table = {
            "ENCLOSURE": self._cmd_ENCLOSURE,
            "DOME":      self._cmd_DOME,
        }

    def _simulate_shutter_motion(self, mech, state):
        duration = self.shutter_motion_time.get(state.upper(), 5.0)
        start = self.shutter_pos[mech]
        target = 90.0 if state == "OPEN" else 0.0
        delta = target - start
        steps = int(duration / 0.1)
        step_size = delta / steps if steps > 0 else 0.0

        for _ in range(steps):
            self.shutter_pos[mech] += step_size
            time.sleep(0.1)

        self.shutter_pos[mech] = target
        self.shutters[mech] = state

    def _simulate_dome_motion(self, new_az):
        delta = abs(new_az - self.dome_az)
        if delta == 0:
            return
        duration = delta / self.dome_az_speed
        steps = int(duration / 0.1)
        step_size = (new_az - self.dome_az) / steps if steps > 0 else 0.0

        for _ in range(steps):
            self.dome_az += step_size
            time.sleep(0.1)

        self.dome_az = new_az

    def _cmd_ENCLOSURE(self, cmd_id, params):
        mech = (params.get("mechanism") or "ENCLOSURE").upper()
        state = (params.get("state") or "CLOSE").upper()

        if mech not in self.shutters:
            self.send_result("ERROR", cmd_id, error_code="BADPARAM")
            return

        self.send_result("ACK", cmd_id)
        self._simulate_shutter_motion(mech, state)
        self.send_result("DONE", cmd_id)

    def _cmd_DOME(self, cmd_id, params):
        try:
            az = float(params["az_angle"])
        except Exception:
            self.send_result("ERROR", cmd_id, error_code="BADPARAM")
            return

        self.send_result("ACK", cmd_id)
        self._simulate_dome_motion(az)
        self.send_result("DONE", cmd_id)

    def handle_command(self, msg):
        cmd_id = msg.get("cmd_id")
        command = (msg.get("command") or "").upper()
        params = msg.get("params", {}) or {}

        handler = self.command_table.get(command)
        if not handler:
            log("ENC", f"Unknown command '{command}'")
            self.send_result("ERROR", cmd_id, error_code="NETUNK")
            return

        handler(cmd_id, params)
