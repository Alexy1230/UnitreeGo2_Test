"read robot state (IMU, pos_motor0) -- low_level"

import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

def LowStateHandler(msg):
    """
    Callback function of the subscriber
    """
    print(f"IMU: {msg.imu_state.rpy}")
    print(f"Motor 0 position: {msg.motor_state[0].q}")
    time.sleep(0.5)

if __name__ == "__main__":
    ChannelFactoryInitialize(0)
    lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_subscriber.Init(LowStateHandler, 10)

    try:
        while True:
            print("Reading robot state.")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stop by Keyboard.")


    