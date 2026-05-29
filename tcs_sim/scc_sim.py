# scc_sim.py
#
# Supervisory Control Computer (SCC) simulator.
#
# - SUB on heartbeat bus: receives JSON heartbeats from ALT, AZ, CAS
# - PUB to TCS bus: sends TELESCOPESUMMARY messages
# - Maintains an in memory dict self.subsystems as the live state
# - Logs snapshots to a CSV file using pandas
#   CSV columns: timestamp, ALT, AZ, CAS, overall
#   Newest snapshot is always the first data row

import time
import json
import threading
import os

import zmq
import pandas as pd

from bus_config import HEARTBEAT_ENDPOINT, SCC_TO_TCS_ENDPOINT, now_ts, log

# CSV log path
LOG_CSV = "./scc_log.csv"


class SCCSimulator:
    def __init__(self):
        self.ctx = zmq.Context.instance()

        # Heartbeat subscriber (from subsystems)
        self.hb_sub = self.ctx.socket(zmq.SUB)
        self.hb_sub.bind(HEARTBEAT_ENDPOINT)
        self.hb_sub.setsockopt_string(zmq.SUBSCRIBE, "heartbeat.")

        # SCC to TCS publisher
        self.scc_pub = self.ctx.socket(zmq.PUB)
        self.scc_pub.connect(SCC_TO_TCS_ENDPOINT)

        # Live state of subsystems in memory
        # name -> { "state": str, "last_heartbeat": int, "current_pos_deg": float }
        self.subsystems = {}

        self.running = True

        # Ensure CSV exists with correct header
        self._init_csv()

        log("SCC", f"SCC will log CSV snapshots to {LOG_CSV}")

    # ------------- CSV and state helpers -------------

    def _init_csv(self):
        """Create CSV with header if it does not exist."""
        if not os.path.exists(LOG_CSV):
            df = pd.DataFrame(
                columns=["timestamp", "ALT", "AZ", "CAS", "overall"]
            )
            df.to_csv(LOG_CSV, index=False)

    def _get_state(self, name: str) -> str:
        """Return latest state string for the given subsystem."""
        info = self.subsystems.get(name)
        if info is None:
            return "UNKNOWN"
        return info.get("state", "UNKNOWN")

    def compute_overall_state(self) -> str:
        """
        Compute overall telescope state from subsystem states.

        Rule:
        - If any subsystem is FAULT -> overall FAULT
        - Else if any subsystem is not IDLE or UNKNOWN -> BUSY
        - Else -> IDLE (or UNKNOWN if nothing known)
        """
        if not self.subsystems:
            return "UNKNOWN"

        states = [self._get_state(n) for n in self.subsystems.keys()]

        if any(s == "FAULT" for s in states):
            return "FAULT"
        if any(s not in ("IDLE", "UNKNOWN") for s in states):
            return "BUSY"
        return "IDLE"

    def log_snapshot_pandas(self):
        """
        Log one snapshot row at the top of the CSV using pandas.

        Row layout:
            timestamp, ALT, AZ, CAS, overall

        Newest row appears immediately after the header.
        """
        ts = now_ts()
        alt_state = self._get_state("ALT")
        az_state = self._get_state("AZ")
        cas_state = self._get_state("CAS")
        overall = self.compute_overall_state()

        new_row = pd.DataFrame(
            [{
                "timestamp": ts,
                "ALT": alt_state,
                "AZ": az_state,
                "CAS": cas_state,
                "overall": overall,
            }]
        )

        # If file exists, read and prepend
        if os.path.exists(LOG_CSV):
            try:
                old_df = pd.read_csv(LOG_CSV)
            except Exception as e:
                log("SCC", f"ERROR reading {LOG_CSV}, recreating: {e}")
                old_df = pd.DataFrame(
                    columns=["timestamp", "ALT", "AZ", "CAS", "overall"]
                )
        else:
            old_df = pd.DataFrame(
                columns=["timestamp", "ALT", "AZ", "CAS", "overall"]
            )

        updated = pd.concat([new_row, old_df], ignore_index=True)

        try:
            updated.to_csv(LOG_CSV, index=False)
        except Exception as e:
            log("SCC", f"ERROR writing {LOG_CSV}: {e}")

    # ------------- Messaging to TCS -------------

    def send_summary(self):
        """
        Send a TELESCOPESUMMARY message to TCS and log a snapshot.

        Body layout:
          {
            "msg_type": "TELESCOPESUMMARY",
            "source": "SCC",
            "timestamp": ...,
            "overall_state": "...",
            "subsystems": { "ALT": {...}, "AZ": {...}, "CAS": {...} }
          }
        """
        topic = "telestatus.overall"
        body = {
            "msg_type": "TELESCOPESUMMARY",
            "source": "SCC",
            "timestamp": now_ts(),
            "overall_state": self.compute_overall_state(),
            "subsystems": self.subsystems,
        }

        # Publish as two frames: topic and JSON
        self.scc_pub.send_string(topic, flags=zmq.SNDMORE)
        self.scc_pub.send_string(json.dumps(body))

        log("SCC", f"SENT TELESCOPESUMMARY: {body}")

        # Also log a snapshot to the CSV
        self.log_snapshot_pandas()

    def summary_loop(self):
        """
        Background thread that sends summary to TCS every few seconds.
        """
        while self.running:
            time.sleep(5.0)
            if self.subsystems:
                self.send_summary()

    # ------------- Main receive loop -------------

    def run(self):
        """
        Main loop for SCC simulator.

        - Receives heartbeat JSON messages from subsystems
        - Updates self.subsystems in memory
        - Logs each heartbeat snapshot to CSV
        - Background thread sends periodic TELESCOPESUMMARY
        """
        log("SCC", f"SCC simulator starting, listening for heartbeat on {HEARTBEAT_ENDPOINT}")
        log("SCC", f"SCC publishing summaries on {SCC_TO_TCS_ENDPOINT}")

        # Start periodic summary thread
        threading.Thread(target=self.summary_loop, daemon=True).start()

        while self.running:
            try:
                topic = self.hb_sub.recv_string()
                body = self.hb_sub.recv_string()
            except KeyboardInterrupt:
                self.running = False
                break

            try:
                msg = json.loads(body)
            except json.JSONDecodeError:
                log("SCC", f"Invalid heartbeat JSON: {body}")
                continue

            source = msg.get("source", "UNKNOWN")
            state = msg.get("state", "UNKNOWN")
            pos = msg.get("current_pos_deg", None)
            ts = msg.get("timestamp", now_ts())

            # Update in memory state
            self.subsystems[source] = {
                "state": state,
                "last_heartbeat": ts,
                "current_pos_deg": pos,
            }

            log("SCC", f"RECV HB from {source}: state={state}, pos={pos}, ts={ts}")

            # Log snapshot for this new heartbeat
            self.log_snapshot_pandas()

        log("SCC", "SCC simulator shutting down.")
