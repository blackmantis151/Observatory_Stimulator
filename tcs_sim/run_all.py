import threading
import time

from bus_config import log
from alt_sim import ALTSubsystem
from az_sim import AZSubsystem
from cas_sim import CASSubsystem
from scc_sim import SCCSimulator


def start_alt():
    sim = ALTSubsystem()
    sim.run()


def start_az():
    sim = AZSubsystem()
    sim.run()


def start_cas():
    sim = CASSubsystem()
    sim.run()


def start_scc():
    sim = SCCSimulator()
    sim.run()


if __name__ == "__main__":
    log("MAIN", "Starting ALT, AZ, CAS, SCC simulators in one process")

    threads = []

    t_alt = threading.Thread(target=start_alt, name="ALT_THREAD", daemon=True)
    t_az  = threading.Thread(target=start_az,  name="AZ_THREAD",  daemon=True)
    t_cas = threading.Thread(target=start_cas, name="CAS_THREAD", daemon=True)
    t_scc = threading.Thread(target=start_scc, name="SCC_THREAD", daemon=True)

    threads.extend([t_alt, t_az, t_cas, t_scc])

    for t in threads:
        t.start()

    log("MAIN", "All simulators started. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        log("MAIN", "KeyboardInterrupt received, exiting.")
        # Threads are daemon=True, process will exit.
