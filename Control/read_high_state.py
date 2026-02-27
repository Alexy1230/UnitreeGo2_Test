"read robot state (position, velocity, yaw_vel) -- high_level"

import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

def SportStateHandler(msg):
    """
    Callback function of the subscriber
    """
    print(f"Position: {msg.position}")
    print(f"Velocity: {msg.velocity}")
    print(f"Yaw speed: {msg.yaw_speed}")

if __name__ == "__main__":
    ChannelFactoryInitialize(0)
    sportstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sportstate_subscriber.Init(SportStateHandler, 10)

    try:
        print("Printing robot state.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stop by Keyboard.")


    