"""
Given gradually increasing velocity commands, let Go2 robot move
"""
import sys
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

def velocity_staircase_test(client):
    print("Robot standup")
    client.StandUp()
    time.sleep(3)
    
    print("Robot balanced stand")
    client.BalanceStand()
    time.sleep(2)

    start_speed = 0.5
    max_speed = 1.0
    step_size = 0.25
    duration_per_step = 3.0

    current_speed = start_speed

    try:
        print("Robot start moving")
        while current_speed <= max_speed:
            print(f"Current vx command: {current_speed:.1f} m/s")
            
            start_time = time.time()
            while time.time() - start_time < duration_per_step:
                # Move(vx, vy, vyaw)
                client.Move(current_speed, 0, 0)
                time.sleep(0.1)  # 10Hz command
            
            current_speed += step_size

        print("Testing ends, robot stop moving")
        client.StopMove()
        time.sleep(2)
        
    except KeyboardInterrupt:
        print("External interrupt.")
        client.StopMove()


if __name__ == "__main__":
    # DDS connection initialization
    # ChannelFactoryInitialize(0, "eth0")
    ChannelFactoryInitialize(0)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    input()

    velocity_staircase_test(sport_client)