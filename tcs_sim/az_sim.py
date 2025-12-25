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

class AZSubsystem(SubsystemBase):
    """
    Simulated AZ axis.
    Supports:
      1. Legacy Point-to-Point (AZIMUTH, UNWRAP_AZ commands)
      2. New Trajectory Tracking (GOTO command)
    """

    def __init__(self):
        super().__init__("AZ")

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
        
        
        # --- PID Control State ---
        self.prev_error = 0.0
        self.integral_error = 0.0

        # --- Command Table ---
        self.command_table = {
            "AZIMUTH":    self._cmd_AZIMUTH,
            "UNWRAP_AZ":  self._cmd_UNWRAP_AZ,
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
                log("AZ", f"GOTO (Trajectory) Mode ACTIVATED for ID {cmd_id}")
            else:
                self.send_result("ERROR", cmd_id=cmd_id, error="FILE_WRITE_FAIL")
        else:
            self.send_result("ERROR", cmd_id=cmd_id, error="BAD_DATA")

    # ============================================================
    # NEW: Physics Loop (The "Brain" for Trajectory)
    # ============================================================
    def _physics_loop(self):
        log(self.name, "Physics Thread Started")
        
        # --- PID TUNING ---
        # Kp: Pulls you to the target (Speed)
        # Ki: Fixes tiny steady offsets (Accuracy)
        # Kd: Predicts the future and applies brakes (Stability)
        Kp = 1.5   
        Ki = 0.02  
        Kd = 8.0   # High D term prevents overshoot (braking)

        while self.running:
            start_time = time.time()
            
            # ONLY run physics if we are in Trajectory Mode
            if self.tracking_active:
                target = self.get_interpolated_target(start_time)
                
                # Hold last known target if data ends
                if target is not None:
                    self.target_pos_deg = target
                
                if target is not None:
                    # 1. Calculate Error
                    error = target - self.current_pos_deg
                    
                    # (FOR AZIMUTH ONLY: Uncomment this block in az_sim.py)
                    if error > 180: error -= 360
                    elif error < -180: error += 360

                    # 2. Calculate Integral (Sum of errors over time)
                    # Anti-windup: Clamp integral so it doesn't grow huge
                    self.integral_error += error * DT
                    self.integral_error = np.clip(self.integral_error, -5.0, 5.0)

                    # 3. Calculate Derivative (Rate of change of error)
                    # If we are closing the gap fast, derivative is negative -> BRAKING
                    derivative = (error - self.prev_error) / DT
                    self.prev_error = error

                    # 4. PID Output (This is our "Ideal Velocity")
                    pid_vel = (Kp * error) + (Ki * self.integral_error) + (Kd * derivative)

                    # 5. Apply Physics Limits (Motor capabilities)
                    # First, clip the requested velocity to the motor's max speed
                    target_vel = np.clip(pid_vel, -MAX_VEL, MAX_VEL)
                    
                    # Second, limit the Acceleration (Jerk)
                    # We can't jump from 0 to 2.0 instantly. We must ramp up.
                    delta_v = target_vel - self.current_vel
                    delta_v = np.clip(delta_v, -MAX_ACCEL * DT, MAX_ACCEL * DT)
                    
                    # Update State
                    self.current_vel += delta_v
                    self.current_pos_deg += self.current_vel * DT
                    
                    # (FOR AZIMUTH ONLY: Uncomment in az_sim.py)
                    self.current_pos_deg = self.current_pos_deg % 360

                    # Sync Legacy Alias
                    self.current_pos = self.current_pos_deg

                    # 6. State Management (Slewing vs Tracking)
                    # Use a slightly tighter tolerance for logic, but keep existing for "Target Reached"
                    if abs(error) < TOLERANCE:
                        self.state = "TRACKING"
                        if not self.target_reached_flag:
                            self.target_reached_flag = True
                            if self.active_cmd_id:
                                self.send_result("COMPLETED", cmd_id=self.active_cmd_id, msg="Target Reached")
                    else:
                        self.state = "SLEWING"

                    # 7. Log & Report
                    self.log_current_state_to_disk(start_time, self.current_pos_deg)
                    self.send_status(cmd_id=self.active_cmd_id, error=error)

            # Timing (Maintain 10Hz)
            elapsed = time.time() - start_time
            time.sleep(max(0, DT - elapsed))

    # ============================================================
    # EXISTING: Legacy Motion Engine (Modified for Safety)
    # ============================================================
    def _start_move(self, target_deg: float, timeout_s: float):
        # --- SAFETY: Disable Trajectory Mode ---
        self.tracking_active = False 
        self.active_cmd_id = None

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

                # Check for overshoot (Simple clamp)
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

        # Legacy limit check (might differ from wrap logic, keeping original)
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
        
    def _cmd_STOP(self, cmd_id, params):
        # Legacy STOP disables Trajectory
        self.tracking_active = False 
        
        mech = (params.get("mechanism") or "").upper()
        if mech not in ("", "AZ"):
            log("AZ", f"STOP received with unexpected mechanism '{mech}', treating as AZ")

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
    sys = AZSubsystem()
    sys.run()