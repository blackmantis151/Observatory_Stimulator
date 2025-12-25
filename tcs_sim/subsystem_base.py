import time
import json
import threading
import pandas as pd
import numpy as np
import csv
import os
import zmq


from bus_config import (
    CMD_ENDPOINT,
    RESULT_ENDPOINT,
    HEARTBEAT_ENDPOINT,
    STATUS_ENDPOINT,
    now_ts,
    log,
)


class SubsystemBase:
    """
    Base class for all subsystems (ALT, AZ, CAS).

    Handles:
      - ZeroMQ socket setup
      - Heartbeat loop
      - Command receive loop
      - Sending ACK / COMPLETED / ERROR / STATUS / HEARTBEAT

    Subclasses must implement:
      - handle_command(self, msg: dict)
    """

    def __init__(self, name: str):
        self.name = name.upper()
        self.running = True
        
        # --- NEW: File Paths & Thread Safety ---
        self.traj_file = f"trajectory_{self.name.lower()}.csv"
        self.status_file = f"status_{self.name.lower()}.csv"
        self.data_lock = threading.Lock()

        # --- NEW: Initialize Files & Recover State ---
        self._init_files()
        
        # Generic state; subclasses can use/extend
        self.state = "IDLE"
        self.current_pos_deg = self._recover_last_position() # <--- CHANGED FROM 0.0
        self.target_pos_deg = 0.0

        self.ctx = zmq.Context.instance()

        # ----- Command SUB (TCS -> subsystem) -----
        self.cmd_sub = self.ctx.socket(zmq.SUB)
        self.cmd_sub.connect(CMD_ENDPOINT)
        topic = f"cmd.{self.name}"
        self.cmd_sub.setsockopt_string(zmq.SUBSCRIBE, topic)

        # ----- Result PUB (subsystem -> TCS) -----
        self.result_pub = self.ctx.socket(zmq.PUB)
        self.result_pub.connect(RESULT_ENDPOINT)

        # ----- Status PUB (subsystem -> TCS) -----
        self.status_pub = self.ctx.socket(zmq.PUB)
        self.status_pub.connect(STATUS_ENDPOINT)

        # ----- Heartbeat PUB (subsystem -> SCC) -----
        self.hb_pub = self.ctx.socket(zmq.PUB)
        self.hb_pub.connect(HEARTBEAT_ENDPOINT)

    # ---------- Generic send helpers ----------

    def send_result(self, msg_type: str, cmd_id=None, **extra):
        topic = f"result.{self.name}"
        body = {
            "msg_type": msg_type,
            "source": self.name,
            "timestamp": now_ts(),
        }
        if cmd_id is not None:
            body["cmd_id"] = cmd_id
        body.update(extra)
        self.result_pub.send_string(topic, flags=zmq.SNDMORE)
        self.result_pub.send_string(json.dumps(body))
        log(self.name, f"SENT on {topic}: {body}")

    def send_status(self, cmd_id=None, **extra):
        topic = f"status.{self.name}"
        body = {
            "msg_type": "STATUS",
            "source": self.name,
            "timestamp": now_ts(),
            "state": self.state,
            "current_pos_deg": self.current_pos_deg,
            "target_pos_deg": self.target_pos_deg,
        }
        if cmd_id is not None:
            body["cmd_id"] = cmd_id
        body.update(extra)  # subclasses can add fields
        self.status_pub.send_string(topic, flags=zmq.SNDMORE)
        self.status_pub.send_string(json.dumps(body))
        log(self.name, f"SENT on {topic}: {body}")

    def send_heartbeat(self):
        topic = f"heartbeat.{self.name}"
        body = {
            "msg_type": "HEARTBEAT",
            "source": self.name,
            "timestamp": now_ts(),
            "state": self.state,
            "current_pos_deg": self.current_pos_deg,
        }
        self.hb_pub.send_string(topic, flags=zmq.SNDMORE)
        self.hb_pub.send_string(json.dumps(body))
        # log(self.name, f"SENT HB on {topic}: {body}")
        
        
        
    # ---------- File & Math Helpers (NEW) ----------

    def _init_files(self):
        """Ensures CSV files exist with correct headers."""
        # Trajectory File
        if not os.path.exists(self.traj_file):
            with open(self.traj_file, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'position'])
        
        # Status File (Append-only)
        if not os.path.exists(self.status_file):
            with open(self.status_file, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'position'])

    def _recover_last_position(self):
        """Reads the last line of the status CSV to recover state on restart."""
        default_pos = 90.0 if self.name == 'ALT' else 0.0
        
        if not os.path.exists(self.status_file):
            return default_pos

        try:
            with open(self.status_file, "r") as f:
                # Efficiently seek to the end
                f.seek(0, os.SEEK_END)
                if f.tell() < 5: return default_pos # Empty file check
                
                pos = f.tell() - 2
                while pos > 0 and f.read(1) != "\n":
                    pos -= 1
                    f.seek(pos, os.SEEK_SET)
                last_line = f.readline().strip()
                
                parts = last_line.split(',')
                if len(parts) >= 2 and parts[0].replace('.','',1).isdigit():
                    recovered = float(parts[1])
                    log(self.name, f"Recovered Position from Disk: {recovered}")
                    return recovered
        except Exception as e:
            log(self.name, f"Error recovering state: {e}")
        
        return default_pos
    
    
    # Add this inside class SubsystemBase in subsystem_base.py
    def clear_trajectory_data(self):
        """Wipes the existing trajectory buffer."""
        with self.data_lock: # Ensure thread safety
            self.traj_timestamps = []
            self.traj_positions = []
            self.traj_t_start = 0.0
            self.traj_t_end = 0.0
            log(self.name, "Trajectory buffer cleared.")
            
    

    def append_trajectory_data(self, timestamps, positions):
        """Thread-safe method to append new trajectory packets."""
        try:
            with self.data_lock:
                with open(self.traj_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    for t, p in zip(timestamps, positions):
                        writer.writerow([t, p])
            return True
        except Exception as e:
            log(self.name, f"Error appending trajectory: {e}")
            return False

    def log_current_state_to_disk(self, timestamp, position):
        """Persist status to CSV (Distinct from network send_status)."""
        try:
            with open(self.status_file, 'a', newline='') as f:
                csv.writer(f).writerow([f"{timestamp:.3f}", f"{position:.4f}"])
        except Exception:
            pass 

    def get_interpolated_target(self, query_time):
        """Reads trajectory CSV and returns the target position for Time T."""
        try:
            with self.data_lock:
                # Note: In high-performance scenarios, cache the DataFrame in memory
                if not os.path.exists(self.traj_file): return None
                df = pd.read_csv(self.traj_file)
            
            times = df['timestamp'].values
            if len(times) == 0: return None
            
            # Binary search
            idx = np.searchsorted(times, query_time)
            
            # Boundary checks
            if idx == 0: return df.iloc[0]['position']
            if idx >= len(times): return df.iloc[-1]['position']
            
            # Linear Interpolation
            t1, t2 = times[idx-1], times[idx]
            p1, p2 = df.iloc[idx-1]['position'], df.iloc[idx]['position']
            
            if t2 - t1 == 0: return p1
            
            fraction = (query_time - t1) / (t2 - t1)
            target = p1 + (p2 - p1) * fraction
            return target
            
        except Exception:
            return None

    # ---------- Background loops ----------

    def heartbeat_loop(self):
        while self.running:
            self.send_heartbeat()
            time.sleep(1.0)

    # ---------- To be overridden ----------

    def handle_command(self, msg: dict):
        cmd_id = msg.get("cmd_id")
        command = (msg.get("command") or "").upper()
        params = msg.get("params", {}) or {}

        handler = getattr(self, "command_table", {}).get(command)
        if not handler:
            self.send_result("ERROR", cmd_id=cmd_id, error_code="NETUNK")
            log(self.name, f"Unknown command '{command}'")
            return

        handler(cmd_id, params)

    # ---------- Main loop ----------

    def run(self):
        log(self.name, f"{self.__class__.__name__} starting, listening for commands on {CMD_ENDPOINT}")

        hb_thread = threading.Thread(target=self.heartbeat_loop, daemon=True)
        hb_thread.start()

        while self.running:
            try:
                topic = self.cmd_sub.recv_string()
                body_str = self.cmd_sub.recv_string()
            except KeyboardInterrupt:
                break

            try:
                msg = json.loads(body_str)
            except json.JSONDecodeError:
                log(self.name, f"Invalid JSON from TCS: {body_str}")
                continue

            log(self.name, f"RECV on {topic}: {msg}")

            if msg.get("msg_type") != "CMD":
                log(self.name, "Ignoring non-CMD message")
                continue

            # delegate to subclass
            self.handle_command(msg)

        self.running = False
        log(self.name, f"{self.__class__.__name__} shutting down.")
