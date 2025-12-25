# cas_sim.py — FINAL corrected version for TCS core

import threading
import time
import json
import numpy as np  # Needed for physics limits

from subsystem_base import SubsystemBase
from bus_config import log

# --- CONFIGURATION ---
MAX_VEL = 2.0       # deg/s (Matches your existing limit)
MAX_ACCEL = 0.3     # deg/s^2
DT = 0.1            # 10Hz Physics Loop
TOLERANCE = 0.05    # Degrees for "Target Reached"

class CASSubsystem(SubsystemBase):
    """
    Simulated Cassegrain Rotator
    Supports:
        1. Legacy Point-to-Point (CAS <deg>, UNWRAP_CAS)
        2. New Trajectory Tracking (GOTO command)
    """

    def __init__(self):
        super().__init__("CAS")

        # --- Legacy State Variables ---
        self.current_pos_deg = 0.0
        self.target_pos_deg = 0.0
        # Sync legacy variable
        self.current_pos = self.current_pos_deg

        self.state = "STOPPED"
        self.last_error = "OK"

        # --- Motion Parameters ---
        self.vel_limit_deg_s = 2.0
        self.sim_speed = 1.0

        # --- Thread Control ---
        self._motion_thread = None
        self._motion_stop = False
         
         
        # --- PID Control State ---
        self.prev_error = 0.0
        self.integral_error = 0.0
        
        # --- NEW: Trajectory Control Flags ---
        self.tracking_active = False
        self.active_cmd_id = None
        self.target_reached_flag = False
        self.current_vel = 0.0

        # --- Command Table ---
        self.command_table = {
            "CAS":        self._cmd_CAS,
            "UNWRAP_CAS": self._cmd_UNWRAP_CAS,
            "STOP":       self._cmd_STOP,
            "GOTO":       self._cmd_GOTO,    # <--- NEW
            "GOTOF":      self._cmd_GOTO,
        }

        # --- NEW: Start Physics Thread (Always runs in background) ---
        self.physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.physics_thread.start()

    # ============================================================
    # NEW: Trajectory Command Handler (GOTO)
    # ============================================================
    def _cmd_GOTO(self, cmd_id, params):
        """
        Handles incoming JSON trajectory packets.
        Expected params: {'timestamp': [t1...], 'position': [p1...]}
        """
        # 1. Stop Legacy Motion (if running)
        self._motion_stop = True
        if self._motion_thread and self._motion_thread.is_alive():
            self._motion_thread.join(timeout=0.2)
        
       
        
        # 2. Send ACK
        self.send_result("ACK", cmd_id=cmd_id)

        # 3. Append Data
        timestamps = params.get('timestamp')
        positions = params.get('position')
         
        
        if timestamps and positions:
            success = self.append_trajectory_data(timestamps, positions)
            if success:
                # 4. Activate Physics Loop
                self.active_cmd_id = cmd_id
                self.target_reached_flag = False 
                self.tracking_active = True
                self.unwrapping = False
                log("AZ", f"GOTO (Trajectory) Mode ACTIVATED for ID {cmd_id}")
            else:
                self.send_result("ERROR", cmd_id=cmd_id, error="FILE_WRITE_FAIL")
        else:
            self.send_result("ERROR", cmd_id=cmd_id, error="BAD_DATA")

    # ============================================================
    # NEW: Physics Loop (The "Brain" for Trajectory)
    # ============================================================
    def _physics_loop(self):
        log("CAS", "Physics Thread Started")
        
        while self.running:
            start_time = time.time()
            
            # ONLY run physics if we are in Trajectory Mode
            if self.tracking_active:
                # 1. Get Target from Base Class
                target = self.get_interpolated_target(start_time)
                # Hold last known target if data ends
                if target is not None:
                    self.target_pos_deg = target
                
                if target is not None:
                    # 2. Calculate Error
                    # Note: CAS usually doesn't wrap 0-360 automatically 
                    # like AZ unless specified. We use simple linear error.
                    error = target - self.current_pos_deg
                    
                    # 3. Calculate Physics (P-Control + Limits)
                    cmd_vel = np.clip(error * 1.5, -MAX_VEL, MAX_VEL)
                    
                    delta_v = cmd_vel - self.current_vel
                    delta_v = np.clip(delta_v, -MAX_ACCEL * DT, MAX_ACCEL * DT)
                    
                    self.current_vel += delta_v
                    self.current_pos_deg += self.current_vel * DT
                    
                    # Update Legacy Alias
                    self.current_pos = self.current_pos_deg

                    # 4. Check Completion
                    if abs(error) < TOLERANCE and not self.target_reached_flag:
                        self.target_reached_flag = True
                        if self.active_cmd_id:
                            self.send_result("COMPLETED", cmd_id=self.active_cmd_id, msg="Target Reached")
                        self.state = "TRACKING"

                    # 5. Log & Report (Status Bus)
                    self.log_current_state_to_disk(start_time, self.current_pos_deg)
                    self.send_status(cmd_id=self.active_cmd_id, error=error)
                
            # Timing (Maintain 10Hz)
            elapsed = time.time() - start_time
            time.sleep(max(0, DT - elapsed))

    # ============================================================
    # EXISTING: Legacy Motion Engine (Modified for Safety)
    # ============================================================
    def _start_move(self, target_deg, timeout_s):
        # --- SAFETY: Disable Trajectory Mode ---
        self.tracking_active = False 
        self.active_cmd_id = None

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
                # Check for STOP or Trajectory Override
                if self._motion_stop or not self.running or self.tracking_active:
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

                # Clamp logic
                if direction > 0 and self.current_pos_deg > target_deg:
                    self.current_pos_deg = target_deg
                if direction < 0 and self.current_pos_deg < target_deg:
                    self.current_pos_deg = target_deg

                self.send_status()
                time.sleep(0.10)

        self._motion_thread = threading.Thread(target=_loop, daemon=True)
        self._motion_thread.start()
        return {"status": "OK", "error": "OK"}

    # ============================================================
    # EXISTING: Helpers & Handlers
    # ============================================================
    def _set_error(self, code):
        self.last_error = code
        self.state = "ERROR"

    def _reject_if_blocked(self):
        if self.state in ("OFF-LINE", "OVERRIDE", "UNKNOWN"):
            self._set_error("STATEREJECT")
            return "STATEREJECT"
        return None

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
        block = self._reject_if_blocked()
        if block:
            self.send_result("ERROR", cmd_id=cmd_id, error_code=block)
            return
        res = self._start_move(angle, timeout_s=120)
        if res["status"] == "ERROR":
            self.send_result("ERROR", cmd_id=cmd_id, error_code=res["error"])
            return
        log("CAS", f"Accepted CAS {angle:.3f}")

    def _cmd_UNWRAP_CAS(self, cmd_id, params):
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
        log("CAS", f"UNWRAP → {target:.3f}")

    def _cmd_STOP(self, cmd_id, params):
        # Legacy STOP disables Trajectory
        self.tracking_active = False 
        
        mech = (params.get("mechanism") or "").upper()
        if mech not in ("", "CAS"):
            log("CAS", f"STOP received with unexpected mechanism '{mech}', treating as CAS")

        self._motion_stop = True
        if not (self._motion_thread and self._motion_thread.is_alive()):
            self.state = "STOPPED"
            self.last_error = "OK"
            self.send_status()
        self.send_result("ACK", cmd_id=cmd_id)

    def handle_command(self, msg):
        cmd_id = msg.get("cmd_id")
        cmd = (msg.get("command") or "").upper()
        params = msg.get("params", {}) or {}

        f = self.command_table.get(cmd)
        if not f:
            self.send_result("ERROR", cmd_id=cmd_id, error_code="NETUNK")
            return
        f(cmd_id, params)

if __name__ == "__main__":
    sys = CASSubsystem()
    sys.run()