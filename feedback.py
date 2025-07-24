import json

def make_status(status, **kwargs):
    """Common feedback packet."""
    pkt = {'status': status}
    pkt.update(kwargs)
    return json.dumps(pkt).encode()
