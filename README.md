# Observatory Stimulator

Simulates the TCS motors and subsystems over UDP.  
When TCS fires a packet, the stimulator dispatches it to a command‐specific handler, simulates the mechanical response over time, and sends back status/feedback.

## Structure

- `main.py` – entry point, sets up UDP and dispatcher  
- `config/` – YAML settings (ports, timing, speeds)  
- `state.py` – central simulated state (ALT, AZ, CAS, clock)  
- `udp.py` – low‐level UDP send/receive  
- `dispatcher.py` – routes incoming packets to handlers  
- `handlers/` – one module per command (e.g. `goto`, `focus`)  
- `feedback.py` – common feedback‐formatting utilities  
- `tests/` – pytest stubs

## Quickstart

```bash
pip install -r requirements.txt
python main.py
