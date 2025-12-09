import time
from datetime import datetime

# For the stimulator we use localhost everywhere
TCS_IP = "127.0.0.1"
SCC_IP = "127.0.0.1"

# ZeroMQ endpoints (buses)
CMD_ENDPOINT        = f"tcp://{TCS_IP}:5555"   # TCS -> subsystems (CMD)
RESULT_ENDPOINT     = f"tcp://{TCS_IP}:5556"   # subsystems -> TCS (ACK/COMPLETED/ERROR)
HEARTBEAT_ENDPOINT  = f"tcp://{SCC_IP}:5557"   # subsystems -> SCC (HEARTBEAT)
SCC_TO_TCS_ENDPOINT = f"tcp://{SCC_IP}:5558"   # SCC -> TCS (TELESCOPESUMMARY)
STATUS_ENDPOINT     = f"tcp://{TCS_IP}:5559"   # subsystems -> TCS (STATUS telemetry)
# TCS -> SCC bus (5560) can be added later if needed.

def now_ts() -> int:
    """Return current time as Unix timestamp (seconds)."""
    return int(time.time())

def log(role: str, msg: str) -> None:
    """Simple timestamped log."""
    print(f"[{datetime.now().strftime('%H:%M:%S')}] [{role}] {msg}", flush=True)
