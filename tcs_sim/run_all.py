import threading
import time
import matplotlib.pyplot as plt
import numpy as np

from bus_config import log
from alt_sim import ALTSubsystem
from az_sim import AZSubsystem
from cas_sim import CASSubsystem
from scc_sim import SCCSimulator
from amn_sim import AMNSubsystem
from enc_sim import ENCSubsystem

def live_sky_plot(alt_obj, az_obj):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(6, 6))
    
    # --- 1. Setup Standard Sky Chart Orientation ---
    ax.set_theta_zero_location('N')      # 0° azimuth (North) at Top
    ax.set_theta_direction(-1)           # Clockwise (N -> E -> S -> W)
    
    # Zenith (90°) at Center (r=0), Horizon (0°) at Edge (r=90)
    ax.set_rlim(0, 90)                   
    ax.set_yticks([0, 30, 60, 90])       # Draw rings
    ax.set_yticklabels(['90°', '60°', '30°', '0°']) # Label them correctly
    
    ax.set_title("Live Telescope Tracking\n(Center=Zenith, Edge=Horizon)")

    # --- 2. Create Plot Objects ---
    # Blue Line: Trail of where we have been
    trail, = ax.plot([], [], 'b-', linewidth=1, alpha=0.5, label='Trail')
    
    # Red Dot: Where the telescope IS right now
    current_point, = ax.plot([], [], 'ro', markersize=8, label='Actual Pos')
    
    # Green Star: Where the telescope WANTS to be (Target)
    target_point, = ax.plot([], [], 'g*', markersize=12, label='Target Pos')
    
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))

    # Data buffers for the trail
    theta_list = []
    r_list = []

    plt.ion()
    plt.show()

    log("PLOT", "Live plot started.")

    while True:
        try:
            # --- 3. Read Data (Thread-Safe enough for plotting) ---
            # Actual Position
            curr_alt = alt_obj.current_pos_deg
            curr_az  = az_obj.current_pos_deg
            
            # Target Position (The command/trajectory)
            targ_alt = alt_obj.target_pos_deg
            targ_az  = az_obj.target_pos_deg

            # --- 4. Convert to Polar Coordinates ---
            # r = 90 - Altitude (So 90 becomes 0 at center)
            r_curr = 90.0 - curr_alt
            theta_curr = np.radians(curr_az)
            
            r_targ = 90.0 - targ_alt
            theta_targ = np.radians(targ_az)

            # --- 5. Update Trail ---
            # Only add to trail if position changed slightly (optimization)
            if not theta_list or abs(theta_list[-1] - theta_curr) > 0.01 or abs(r_list[-1] - r_curr) > 0.1:
                theta_list.append(theta_curr)
                r_list.append(r_curr)
                # Keep trail short (last 100 points) so it doesn't get messy
                if len(theta_list) > 100:
                    theta_list.pop(0)
                    r_list.pop(0)

            # --- 6. Draw ---
            trail.set_data(theta_list, r_list)
            current_point.set_data([theta_curr], [r_curr])
            target_point.set_data([theta_targ], [r_targ])

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            
            time.sleep(0.1) # 10Hz Refresh Rate

        except KeyboardInterrupt:
            print("[Plot] Interrupted")
            break
        except Exception as e:
            print(f"[Plot] Error: {e}")
            break
            
    plt.close(fig)

if __name__ == "__main__":
    log("MAIN", "Starting Simulators...")

    # 1. Initialize Subsystems (Physics threads start automatically in __init__)
    alt = ALTSubsystem()
    az  = AZSubsystem()
    cas = CASSubsystem()
    scc = SCCSimulator()
    amn = AMNSubsystem()
    enc = ENCSubsystem()

    # 2. Start ZMQ Listener Threads
    # Note: Physics loops are already running in background from __init__
    threads = [
        threading.Thread(target=alt.run, daemon=True),
        threading.Thread(target=az.run,  daemon=True),
        threading.Thread(target=cas.run, daemon=True),
        threading.Thread(target=scc.run, daemon=True),
        threading.Thread(target=amn.run, daemon=True),
        threading.Thread(target=enc.run, daemon=True),
    ]

    for t in threads:
        t.start()

    log("MAIN", "All Systems GO. Launching GUI...")
    
    # 3. Run Plot on Main Thread (Matplotlib requires this)
    try:
        live_sky_plot(alt, az)
    except KeyboardInterrupt:
        log("MAIN", "Shutting down...")