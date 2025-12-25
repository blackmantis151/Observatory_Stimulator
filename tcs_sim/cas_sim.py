import threading
import time
import json
import numpy as np
from subsystem_base import SubsystemBase
from bus_config import log

# --- CONFIGURATION ---
MAX_VEL = 2.0       # deg/s
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
        self.current_pos = self.current_pos_deg

        self.state = "STOPPED"
        self.last_error = "OK"

        # --- Motion Parameters ---
        self.vel_limit_deg_s = 2.0
        self.sim_speed = 1.0

        # --- Thread Control ---
        self._motion_thread = None
        self._motion_stop = False
        
        # --- Trajectory Control Flags ---
        self.tracking_active = False
        self.active_cmd_id = None
        self.target_reached_flag = False
        self.current_vel = 0.0
        
        # --- PID Control State ---
        self.prev_error = 0.0
        self.integral_error = 0.0

        # --- Command Table ---
        self.command_table = {
            "CAS":        self._cmd_CAS,
            "UNWRAP_CAS": self._cmd_UNWRAP_CAS,
            "STOP":       self._cmd_STOP,
            "GOTO":       self._cmd_GOTO,
            "GOTOF":      self._cmd_GOTO,
        }

        self.physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.physics_thread.start()

    # ============================================================
    # NEW: Trajectory Command Handler (GOTO)
    # ============================================================
    def _cmd_GOTO(self, cmd_id, params):
        """
        Handles incoming JSON trajectory packets.
        Wipes previous data to ensure a fresh start.
        """
        # 1. PAUSE EVERYTHING
        self.tracking_active = False
        self._motion_stop = True
        if self._motion_thread and self._motion_thread.is_alive():
            self._motion_thread.join(timeout=0.2)
        
        self.state = "SLEWING"
        self.send_result("ACK", cmd_id=cmd_id)

        timestamps = params.get('timestamp')
        positions = params.get('position')
        
        if timestamps and positions:
            # 2. WIPE OLD DATA (Crucial Fix)
            self.clear_trajectory_data()

            # 3. LOAD NEW DATA
            success = self.append_trajectory_data(timestamps, positions)
            
            if success:
                self.active_cmd_id = cmd_id
                self.target_reached_flag = False 
                
                # 4. RESUME TRACKING
                self.tracking_active = True 
                log("CAS", f"GOTO (Trajectory) Mode ACTIVATED for ID {cmd_id}")
            else:
                self.send_result("ERROR", cmd_id=cmd_id, error="FILE_WRITE_FAIL")
        else:
            self.send_result("ERROR", cmd_id=cmd_id, error="BAD_DATA")

    # ============================================================
    # NEW: Physics Loop (PID Controller)
    # ============================================================
    def _physics_loop(self):
        log("CAS", "Physics Thread Started")
        
        # PID Constants
        Kp = 1.5   
        Ki = 0.02  
        Kd = 8.0   

        while self.running:
            start_time = time.time()
            
            if self.tracking_active:
                target = self.get_interpolated_target(start_time)
                
                if target is not None:
                    self.target_pos_deg = target
                    
                    # 1. Calculate Error
                    # CAS generally doesn't wrap like AZ (0-360), so simple difference
                    error = target - self.current_pos_deg
                    
                    # 2. PID Control
                    # Integral
                    self.integral_error += error * DT
                    self.integral_error = np.clip(self.integral_error, -5.0, 5.0)
                    
                    # Derivative
                    derivative = (error - self.prev_error) / DT
                    self.prev_error = error

                    # Output Velocity
                    pid_vel = (Kp * error) + (Ki * self.integral_error) + (Kd * derivative)

                    # 3. Apply Limits
                    cmd_vel = np.clip(pid_vel, -MAX_VEL, MAX_VEL)
                    
                    delta_v = cmd_vel - self.current_vel
                    delta_v = np.clip(delta_v, -MAX_ACCEL * DT, MAX_ACCEL * DT)
                    
                    self.current_vel += delta_v
                    self.current_pos_deg += self.current_vel * DT
                    self.current_pos = self.current_pos_deg 

                    # 4. Check Completion
                    if abs(error) < TOLERANCE:
                        self.state = "TRACKING"
                        if not self.target_reached_flag:
                            self.target_reached_flag = True
                            if self.active_cmd_id:
                                self.send_result("COMPLETED", cmd_id=self.active_cmd_id, msg="Target Reached")
                    else:
                        self.state = "SLEWING"

                    self.log_current_state_to_disk(start_time, self.current_pos_deg)
                    self.send_status(cmd_id=self.active_cmd_id, error=error)
            
            elapsed = time.time() - start_time
            time.sleep(max(0, DT - elapsed))

    # ============================================================
    # EXISTING: Legacy Motion Engine
    # ============================================================
    def _start_move(self, target_deg, timeout_s):
        # Disable Trajectory
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