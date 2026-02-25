"Generate a rectangle trajectory, the robot move based on that"

import time
import math
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

def rectangle_open_loop(sc):
    # 30cm length for a tile
    # default unit: m
    L, W = 0.3*6, 0.3*6        # 1.8 x 1.8m
    v_const = 0.3              # 0.3 m/s
    
    t_long = L / v_const
    t_wide = W / v_const
    
    phases = [
        (v_const, 0, t_long),  # front
        (0, v_const, t_wide),  # left
        (-v_const, 0, t_long), # back
        (0, -v_const, t_wide)  # right
    ]

    for vx, vy, duration in phases:
        start_time = time.time()
        while time.time() - start_time < duration:
            sc.Move(vx, vy, 0)
            time.sleep(0.05) # 20Hz

        sc.StopMove()
        time.sleep(0.5)

    print("Movement Finished.")


if __name__ == "__main__":
    # DDS connection initialization
    # ChannelFactoryInitialize(0, "eth0")
    ChannelFactoryInitialize(0)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    input()
