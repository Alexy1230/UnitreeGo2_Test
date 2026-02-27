"read L1 lidar (unitree lidar)"

import time
import threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

data = []
class PointCloudTest:
    def __init__(self):
        self.lock = threading.Lock()
        self.save_one_data = True
        self.data = None

    def PointCloudHandler(self, msg):
        """
        Callback function of the subscriber
        """
        print("Received a raw cloud here!")
        with self.lock:
            self.data = msg.data
        print(f"stamp: {msg.header.stamp.sec}")
        print(f"frame: {msg.header.frame_id}")
        print(f"dimension: {msg.width} x {msg.height}")
        print(f"point step: {msg.point_step}")

        print("point fields:")
        for i, field in enumerate(msg.fields):
            print(f"[{i}] name: {field.name:10} | offset: \
                  {field.offset:2} | datatype: {field.datatype} | count: {field.count}")
        print("\n")
        print("\n")
    
    def SaveOneFrame(self):
        if self.save_one_data and self.data is not None:
            self.save_raw_lidar()
            self.save_one_data = False
            print("One frame saved.")
    
    def save_raw_lidar(self):
        timestamp = int(time.time() * 1000)
        with self.lock:
            raw_bytes = bytearray(self.data)
        
        with open(f"lidar_{timestamp}.bin", "wb") as f:
            f.write(raw_bytes)


if __name__ == "__main__":
    ChannelFactoryInitialize(0)

    point_cloud_test = PointCloudTest()
    sportstate_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
    sportstate_subscriber.Init(point_cloud_test.PointCloudHandler, 10)

    try:
        print("Printing robot state.")
        while True:
            point_cloud_test.SaveOneFrame()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stop by Keyboard.")


    