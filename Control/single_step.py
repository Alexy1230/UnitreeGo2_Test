import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.utils.thread import RecurrentThread

ChannelFactoryInitialize(0, "eth0")
time.sleep(2.0)
input("Press Enter to continue...")

sport_client = SportClient()
sport_client.SetTimeout(10.0)
sport_client.Init()
time.sleep(1.0)

sport_client.RecoveryStand()
time.sleep(3.0)

# ✅ Use RecurrentThread instead of a manual loop
def send_move():
    sport_client.Move(0.0, 0.0, 0.3)

move_thread = RecurrentThread(interval=0.02, target=send_move, name="move")
move_thread.Start()

# Run for 3 seconds then stop
time.sleep(3.0)
move_thread.Wait()  # stops the thread

sport_client.StopMove()
time.sleep(1.0)
print("Done.")