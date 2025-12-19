import time
import json
import threading

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

        # Generic state; subclasses can use/extend
        self.state = "IDLE"
        self.current_pos_deg = 0.0
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
