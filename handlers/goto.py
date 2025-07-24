import asyncio, time
from .base import BaseHandler
from feedback import make_status

class GotoHandler(BaseHandler):
    """
    Simulates a GOTO trajectory:
    receives a list of waypoints with timestamps,
    advances state in delta_t chunks,
    sends back statuses.
    """

    async def handle(self, msg):
        """
        msg example:
        {
          "command": "GOTO",
          "trajectory": [
            {"alt": xx, "az": yy, "cas": zz, "time": t0},
            ...
          ]
        }
        """
        traj = msg['trajectory']
        # run simulation
        for wp in traj:
            now = time.time()
            wait = wp['time'] - now
            if wait > 0:
                await asyncio.sleep(wait)
            # advance state in one step
            self.state.update_towards(wp['alt'], wp['az'], wp['cas'], wait or 0)
            fb = self.state.get_feedback()
            # determine status
            status = 'IN POSITION' if (fb['alt']==wp['alt'] and fb['az']==wp['az']) else 'SLEWING'
            yield make_status(status, **fb)
        # finally start tracking
        yield make_status('TRACKING', **self.state.get_feedback())
