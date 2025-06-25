import msgpack
import numpy as np
import zenoh
from arca_sim.state import LowLevelCommand, LowLevelState

z = zenoh.open(zenoh.Config())

# HL side
# def ll_state_cb(sample):
#     ll_state = msgpack.unpackb(sample.payload)
#     print(ll_state)
    
ll_cmd_pub = z.declare_publisher("ll_cmd")
# ll_state_sub = z.declare_subscriber("ll_state", ll_state_cb)

# LL side
def ll_cmd_cb(sample):
    print("received")
    ll_cmd = LowLevelCommand.from_bytes(sample.payload.to_bytes())
    print(ll_cmd)

ll_cmd_sub = z.declare_subscriber("ll_cmd", ll_cmd_cb)
# ll_state_pub = z.declare_publisher("ll_state")

def hl_loop():
    for i in range(100):
        ll_cmd = LowLevelCommand(
            pos=np.full(6, i),
            vel=np.full(6, i),
            current=np.full(6, i),
            kp=np.full(6, i),
            kd=np.full(6, i)
        )
        ll_cmd_pub.put(ll_cmd.to_bytes())
        print("sent")
    
# def ll_loop():
#     ll_state = LowLevelState(
#         gyro=np.zeros(3),
#         quat=np.zeros(4),
#         pos=np.zeros(3),
#         vel=np.zeros(3),
#         current=np.zeros(12)
#     )
    
#     ll_state_pub.put(ll_state.to_bytes())



hl_loop()
# ll_loop()
