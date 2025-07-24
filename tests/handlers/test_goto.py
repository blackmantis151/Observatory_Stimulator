import pytest, asyncio, time
from handlers.goto import GotoHandler
from state import SimulatorState

@pytest.mark.asyncio
async def test_goto_handler():
    state = SimulatorState({'motor_speed_deg_per_s': 1000})
    handler = GotoHandler(state)
    now = time.time()
    traj = [
        {'alt':10,'az':20,'cas':30,'time': now + 0.01},
        {'alt':11,'az':21,'cas':31,'time': now + 0.02},
    ]
    packets = []
    async for pkt in handler.handle({'trajectory': traj}):
        packets.append(pkt)
    # should at least end with TRACKING
    assert any(b'TRACKING' in p for p in packets)
