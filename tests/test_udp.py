import asyncio, pytest
from udp import UDPServer

@pytest.mark.asyncio
async def test_udp_echo(tmp_path):
    received = []

    async def cb(data, addr):
        received.append(data)

    server = UDPServer('127.0.0.1', 9999, cb)
    # start server
    task = asyncio.create_task(server.serve_forever())
    # send a packet
    tr = asyncio.get_running_loop().create_datagram_endpoint(
        lambda: asyncio.DatagramProtocol(), remote_addr=('127.0.0.1',9999))
    transport, _ = await tr
    transport.sendto(b'test')
    await asyncio.sleep(0.1)
    transport.close()
    task.cancel()
    assert received and received[0] == b'test'
