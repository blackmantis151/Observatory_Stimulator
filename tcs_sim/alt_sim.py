import threading
import time
import json
import numpy as np  # Needed for physics limits

from subsystem_base import SubsystemBase
from bus_config import log

# --- CONFIGURATION ---
MAX_VEL = 2.0       # deg/s
MAX_ACCEL = 0.3     # deg/s^2
DT = 0.1            # 10Hz Physics Loop
TOLERANCE = 0.05    # Degrees for "Target Reached"

class ALTSubsystem(SubsystemBase):
    """
    Simulated ALT axis.
    Supports:
      1. Legacy Point-to-Point (ALTITUDE command)
      2. New Trajectory Tracking (GOTO command)
    """

    def __init__(self):
        super().__init__("ALT")

        # --- Legacy State Variables ---
        self.state = "STOPPED"
        self.last_error = "OK"

        # --- Motion Parameters ---
        self.vel_limit_deg_s = 2.0
        self.sim_speed = 1.0

        # --- Thread Control ---
        self._motion_thread = None
        self._motion_stop = False
        
        # --- NEW: Trajectory Control Flags ---
        self.tracking_active = False
        self.active_cmd_id = None
        self.target_reached_flag = False
        self.current_vel = 0.0

        # --- Command Table ---
        self.command_table = {
            "ALTITUDE": self._cmd_ALTITUDE,
            "STOP":     self._cmd_STOP,
            "GOTO":     self._cmd_GOTO,# <--- RENAMED FROM TRAJECTORY
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
        
        self.state = "TRACKING" # Update Status String
        
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
                log("ALT", f"GOTO (Trajectory) Mode ACTIVATED for ID {cmd_id}")
            else:
                self.send_result("ERROR", cmd_id=cmd_id, error="FILE_WRITE_FAIL")
        else:
            self.send_result("ERROR", cmd_id=cmd_id, error="BAD_DATA")

    # ============================================================
    # NEW: Physics Loop (The "Brain" for Trajectory)
    # ============================================================
    def _physics_loop(self):
        log("ALT", "Physics Thread Started")
        
        while self.running:
            start_time = time.time()
            
            # ONLY run physics if we are in Trajectory Mode
            if self.tracking_active:
                # 1. Get Target from Base Class
                target = self.get_interpolated_target(start_time)
                # If target is None (end of file), hold last known target
                if target is not None:
                    self.target_pos_deg = target
                
                if target is not None:
                    # 2. Calculate Error
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
                    
                    # Send live status packet
                    self.send_status(cmd_id=self.active_cmd_id, error=error)
                
            # Timing (Maintain 10Hz)
            elapsed = time.time() - start_time
            time.sleep(max(0, DT - elapsed))

    # ============================================================
    # EXISTING: Legacy Motion Engine (Modified for Safety)
    # ============================================================
    def _start_move(self, target_deg: float, timeout_s: float):
        # --- SAFETY: Disable Trajectory Mode if Legacy Move starts ---
        self.tracking_active = False 
        self.active_cmd_id = None
        
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
            self.send_status()

            while True:
                # Check for STOP or Trajectory Override
                if self._motion_stop or not self.running or self.tracking_active:
                    self.state = "STOPPED"
                    self.send_status()
                    return

                now = time.time()
                elapsed = now - t0

                if elapsed >= t_required_sim:
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

                self.send_status()
                time.sleep(0.10)

        self._motion_thread = threading.Thread(target=_motion_loop, daemon=True)
        self._motion_thread.start()

        return {"status": "OK", "error": "OK"}

    # ============================================================
    # EXISTING: Helpers & Handlers (Unchanged)
    # ============================================================
    def _set_error(self, code: str):
        self.last_error = code
        self.state = "ERROR"

    def _reject_if_blocked(self):
        if self.state in ("OFF-LINE", "OVERRIDE", "UNKNOWN"):
            self._set_error("STATEREJECT")
            return "STATEREJECT"
        return None

    def _cmd_ALTITUDE(self, cmd_id, params):
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
        if angle < 20.0 or angle > 91.0:
            self.state = "LIMIT"
            self._set_error("BADRANGE")
            self.send_result("ERROR", cmd_id=cmd_id, error_code="BADRANGE")
            return
        block = self._reject_if_blocked()
        if block:
            self.send_result("ERROR", cmd_id=cmd_id, error_code=block)
            return
        res = self._start_move(angle, timeout_s=100.0)
        if res["status"] == "ERROR":
            self.send_result("ERROR", cmd_id=cmd_id, error_code=res["error"])
            return
        log("ALT", f"Accepted ALTITUDE move to {angle:.3f}°")

    def _cmd_STOP(self, cmd_id, params):
        # Legacy STOP also disables Trajectory Mode
        self.tracking_active = False 
        
        mech = (params.get("mechanism") or "").upper()
        if mech not in ("", "ALT"):
            log("ALT", f"STOP received with unexpected mechanism '{mech}', treating as ALT")
        self._motion_stop = True
        if not (self._motion_thread and self._motion_thread.is_alive()):
            self.state = "STOPPED"
            self.last_error = "OK"
            self.send_status()
        self.send_result("ACK", cmd_id=cmd_id)

if __name__ == "__main__":
    sys = ALTSubsystem()
    sys.run()