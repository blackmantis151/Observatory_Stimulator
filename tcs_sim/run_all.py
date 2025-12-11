import threading
import time
import matplotlib.pyplot as plt
import numpy as np

from bus_config import log
from alt_sim import ALTSubsystem
from az_sim import AZSubsystem
from cas_sim import CASSubsystem
from scc_sim import SCCSimulator


def live_sky_plot(alt_obj, az_obj):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_theta_zero_location('N')      # 0° azimuth = top (North)
    ax.set_theta_direction(-1)           # Clockwise azimuth
    ax.set_rlim(90, 0)                   # 0° alt at edge, 90° at center

    ax.set_title("Live Alt-Az Sky Plot")

    # Add compass labels
    ax.set_xticks(np.radians([0, 90, 180, 270]))
    ax.set_xticklabels(['N', 'E', 'S', 'W'])

    trail, = ax.plot([], [], 'b.-', label='Trajectory')
    point, = ax.plot([], [], 'ro', label='Current Pos')
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1.0), borderaxespad=0.)

    theta_list = []
    r_list = []

    plt.ion()
    plt.show()

    # Plot initial (0,0) edge point
    theta_list.append(0.0)  # az = 0°
    r_list.append(90.0)     # alt = 0°

    trail.set_data(theta_list, r_list)
    point.set_data([0.0], [90.0])
    fig.canvas.draw_idle()
    plt.pause(0.01)

    while True:
        try:
            alt = alt_obj.current_pos_deg
            az = az_obj.current_pos_deg
            if alt > 0:
                theta = np.radians(az)
                r =  alt

                theta_list.append(theta)
                r_list.append(r)

                trail.set_data(theta_list, r_list)
                point.set_data([theta], [r])

            fig.canvas.draw_idle()
            plt.pause(0.01)
            time.sleep(0.5)

        except KeyboardInterrupt:
            print("[Plot] Interrupted")
            break
        except Exception as e:
            print(f"[Plot] Error: {e}")
            break


if __name__ == "__main__":
    log("MAIN", "Starting ALT, AZ, CAS, SCC simulators in background threads")

    alt = ALTSubsystem()
    az  = AZSubsystem()
    cas = CASSubsystem()
    scc = SCCSimulator()

    threads = [
        threading.Thread(target=alt.run, daemon=True),
        threading.Thread(target=az.run,  daemon=True),
        threading.Thread(target=cas.run, daemon=True),
        threading.Thread(target=scc.run, daemon=True),
    ]

    for t in threads:
        t.start()

    log("MAIN", "All simulators started. Launching sky plot in main thread.")
    log("MAIN", "Press Ctrl+C in the plot window or terminal to exit.")

    try:
        live_sky_plot(alt, az)
    except KeyboardInterrupt:
        log("MAIN", "KeyboardInterrupt received. Exiting.")
