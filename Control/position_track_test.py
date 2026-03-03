"robot move based on the given position points"

import time
import math
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

if __name__ == "__main__":
    # DDS connection initialization
    # ChannelFactoryInitialize(0, "eth0")
    ChannelFactoryInitialize(0)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    try:
        print("Start moving.")
        sport_client.MoveToPos(0.3, 0, 0)

        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stop by Keyboard.")
        sport_client.StopMove() 
        time.sleep(0.2)

        sport_client.StandDown()
        time.sleep(2.5)
        print("Robot low down")