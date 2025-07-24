import json
from handlers import base, goto, focus

class Dispatcher:
    """Route incoming JSON packets to the right handler."""

    def __init__(self, state):
        self.state = state
        self.handlers = {
            'GOTO': goto.GotoHandler(state),
            'FOCUS': focus.FocusHandler(state),
            # add new commands here
        }

    async def handle_packet(self, data, addr):
        """Parse command type and dispatch."""
        try:
            msg = json.loads(data.decode())
            cmd = msg.get('command')
            handler = self.handlers.get(cmd)
            if handler:
                resp = await handler.handle(msg)
            else:
                resp = {'error': f'Unknown command {cmd}'}
        except Exception as e:
            resp = {'error': str(e)}
        return addr, json.dumps(resp).encode()
