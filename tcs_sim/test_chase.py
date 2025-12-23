# import zmq
# import json
# import time
# import numpy as np
# from bus_config import CMD_ENDPOINT

# def send_chase():
#     ctx = zmq.Context()
#     pub = ctx.socket(zmq.PUB)
#     pub.connect(CMD_ENDPOINT)
#     time.sleep(1) # Wait for connection

#     print("Sending GOTO Trajectory...")
#     now = time.time()
    
#     # Create 60 seconds of motion
#     timestamps = np.arange(now, now + 60, 0.5).tolist()
    
#     # Alt: Rises from 40 to 50
#     alt_pos = [40 + (t - now) * 0.1 for t in timestamps]
    
#     # Az: Moves from 180 to 200
#     az_pos = [180 + (t - now) * 0.3 for t in timestamps]

#     # Send ALT
#     pub.send_string("cmd.ALT", flags=zmq.SNDMORE)
#     pub.send_string(json.dumps({
#         "msg_type": "CMD", "command": "GOTO", "cmd_id": 101,
#         "params": {"timestamp": timestamps, "position": alt_pos}
#     }))
    
#     # Send AZ
#     pub.send_string("cmd.AZ", flags=zmq.SNDMORE)
#     pub.send_string(json.dumps({
#         "msg_type": "CMD", "command": "GOTO", "cmd_id": 102,
#         "params": {"timestamp": timestamps, "position": az_pos}
#     }))
#     print("Sent! Watch the plot.")

# if __name__ == "__main__":
#     send_chase()