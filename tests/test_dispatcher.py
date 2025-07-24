import pytest, json
from dispatcher import Dispatcher
from state import SimulatorState

class DummyHandler:
    def __init__(self, state): pass
    async def handle(self, msg): return b'OK'

def test_dispatcher_unknown(monkeypatch):
    state = SimulatorState({'motor_speed_deg_per_s':1.0})
    disp = Dispatcher(state)
    disp.handlers = {'FOO': DummyHandler(state)}
    addr, resp = asyncio.get_event_loop().run_until_complete(
        disp.handle_packet(json.dumps({'command':'BAR'}).encode(), ('a',0))
    )
    assert b'Unknown command' in resp
