from .base import BaseHandler
from feedback import make_status

class FocusHandler(BaseHandler):
    """Stub for FOCUS simulation."""

    async def handle(self, msg):
        # TODO: implement focus-specific simulation
        return make_status('FOCUS_DONE', focus_value=msg.get('value', 0))
