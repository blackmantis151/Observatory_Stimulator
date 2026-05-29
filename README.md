# Observatory Stimulator

Simulates the TCS motors and subsystems over ZeroMQ.  
When TCS fires a packet, the stimulator dispatches it to a command‐specific handler, simulates the mechanical response over time, and sends back status/feedback.

<!-- ## Structure

- `main.py` – entry point, sets up UDP and dispatcher  
- `config/` – YAML settings (ports, timing, speeds)  
- `state.py` – central simulated state (ALT, AZ, CAS, clock)  
- `udp.py` – low‐level UDP send/receive  
- `dispatcher.py` – routes incoming packets to handlers  
- `handlers/` – one module per command (e.g. `goto`, `focus`)  
- `feedback.py` – common feedback‐formatting utilities  
- `tests/` – pytest stubs -->

## Quickstart

This project uses `uv` for package management
```bash
#clone repo
git clone https://github.com/blackmantis151/Observatory_Stimulator.git

#install uv
curl -LsSf https://astral.sh/uv/install.sh | sh
#sync all packages
uv sync

#run simulator
python tcs_sim/run_all.py
```
