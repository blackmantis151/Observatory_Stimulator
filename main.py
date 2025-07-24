import asyncio
import yaml

from config import settings
from udp import UDPServer
from dispatcher import Dispatcher
from state import SimulatorState

async def main():
    # load config
    cfg = yaml.safe_load(open('config/settings.yaml'))
    state = SimulatorState(cfg['simulator'])
    dispatcher = Dispatcher(state)
    server = UDPServer(
        cfg['udp']['host'],
        cfg['udp']['port'],
        dispatcher.handle_packet
    )
    print(f"Starting stimulator on {cfg['udp']['host']}:{cfg['udp']['port']}")
    await server.serve_forever()

if __name__ == '__main__':
    asyncio.run(main())
