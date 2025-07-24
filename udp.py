import asyncio

class UDPServer:
    """Simple asyncio UDP server."""

    def __init__(self, host, port, packet_callback):
        self.host = host
        self.port = port
        self.packet_callback = packet_callback

    class Proto(asyncio.DatagramProtocol):
        def __init__(self, callback):
            self.callback = callback

        def datagram_received(self, data, addr):
            asyncio.create_task(self.callback(data, addr))

    async def serve_forever(self):
        loop = asyncio.get_running_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: UDPServer.Proto(self.packet_callback),
            local_addr=(self.host, self.port)
        )
        # keep running
        await asyncio.Event().wait()
