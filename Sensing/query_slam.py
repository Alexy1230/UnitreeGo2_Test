" Slam service example"

import json
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.go2.slam.slam_client import SlamClient

def SlamOdoHandler(msg):
    """
    Callback function of the slam-odometry subscriber
    """
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    linear = msg.twist.twist.linear
    angular = msg.twist.twist.angular
    print(f"Position: {[position.x, position.y, position.z]}")
    print(f"Orientation: {[orientation.x, orientation.y, orientation.z, orientation.w]}")
    print(f"Pose variance: {msg.pose.covariance}")
    print(f"Linear vel: {[linear.x, linear.y, linear.z]}")
    print(f"Angular vel: {[angular.x, angular.y, angular.z]}")
    print(f"Twist variance: {msg.twist.covariance}")
    time.sleep(0.05)

if __name__ == "__main__":
    ChannelFactoryInitialize(0)

    map_path = "test_map.pcd"
    slam_client = SlamClient()
    slam_client.SetTimeout(10.0)
    slam_client.Init()

    print("start building map.")
    code = slam_client.StartMapping()
    if code != 0:
        print(f"StartMapping failed, code: {code}")
    else:
        # Auto-build map for 10 seconds
        build_duration = 10
        time.sleep(build_duration)

        # Stop building and save map
        code = slam_client.EndMapping(save_path=map_path)
        if code != 0:
            print(f"EndMapping failed, code: {code}")
        else:
            print(f"Map saved to {map_path}")
    
    time.sleep(1)

    code = slam_client.InitializePose(load_path=map_path)
    if code != 0:
        print(f"Load and InitializePose failed, code: {code}")
    else:
        print("Map loaded and pose initialized.")

    # subscribe slam data
    slamstate_subscriber = ChannelSubscriber("rt/unitree/slam_relocation/odom", Odometry_)
    slamstate_subscriber.Init(SlamOdoHandler, 10)

    try:
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stop by Keyboard.")
        slam_client.CloseSlam()
   
