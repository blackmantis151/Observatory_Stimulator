class BaseHandler:
    """Abstract base for all command handlers."""

    def __init__(self, state):
        self.state = state

    async def handle(self, msg):
        """
        Run simulation for this command.
        Must return bytes to send back via UDP.
        """
        raise NotImplementedError
