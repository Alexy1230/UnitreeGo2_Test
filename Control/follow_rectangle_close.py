"Generate a rectangle trajectory, the robot move based on that, PID closed-loop"

import time
import math
import threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient

class RobotAgent:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s]
        self.kp_ = 0.8
        self.kp_ang_ = 0.5
        self.lock = threading.Lock()

        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.yaw_speed = 0.0
        self.data_received = False

    def Init(self):
        self.sportstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sportstate_subscriber.Init(self.SportStateHandler, 10)

        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

    def SportStateHandler(self, msg):
        with self.lock:
            self.position = msg.position.copy()
            self.velocity = msg.velocity.copy()
            self.yaw_speed = msg.yaw_speed
            yaw = msg.imu_state.rpy[2]
            self.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            self.data_received = True


    def run_rectangle(self):
        # 30cm length for a tile
        # default unit: m
        L, W = 0.3*6, 0.3*6        # 1.8 x 1.8m
        vx_max = 0.3               # 0.3 m/s
        vy_max = 0.1
        vyaw_max = 0.1
        # dest point [x, y, yaw]
        # trajectory = [[L,0,0], [0,0,math.pi/2], [W,0,0],
        #                [0,0,math.pi/2], [L,0,0], [0,0,math.pi/2], [W,0,0], [0,0,math.pi/2]]
        dest_point = [[L,0,0], [L,0,math.pi/2], [L,W,math.pi/2],
                    [L,W,math.pi], [0,W,math.pi], [0,W,math.pi*3/2], [0,0,math.pi*3/2], [0,0,0]]
        current_idx = 0

        while True:
            if not self.data_received:
                time.sleep(0.1)
                continue

            with self.lock: # 确保读取 $x, y, yaw$ 时数据是对齐的
                position = self.position[:]
                yaw = self.yaw

            target = dest_point[current_idx]
            world_error_x = target[0] - position[0]
            world_error_y = target[1] - position[1]

            # transformation
            cos_y = math.cos(-yaw)
            sin_y = math.sin(-yaw)
            body_error_x = world_error_x * cos_y - world_error_y * sin_y
            body_error_y = world_error_x * sin_y + world_error_y * cos_y

            error_dist = math.sqrt(body_error_x**2 + body_error_y**2)
            error_ang = target[2] - yaw
            error_ang = math.atan2(math.sin(error_ang), math.cos(error_ang))


            if error_dist < 0.1 and abs(error_ang) < 0.1:
                # go to the next trajectory
                current_idx = (current_idx + 1) % 8
                time.sleep(0.5)
                if current_idx == 0:
                    print("Movement Finished.")
                    self.sport_client.StopMove()
                    time.sleep(0.5)
                    return
            else:
                vx_desired = body_error_x * self.kp_
                vx_cmd = max(-vx_max, min(vx_max, vx_desired))

                vy_desired = body_error_y * self.kp_
                vy_cmd = max(-vy_max, min(vy_max, vy_desired))

                vyaw_desired = error_ang * self.kp_ang_
                vyaw_cmd = max(-vyaw_max, min(vyaw_max, vyaw_desired))

                self.sport_client.Move(vx_cmd, vy_cmd, vyaw_cmd)
            
            time.sleep(0.05)


if __name__ == "__main__":
    # DDS connection initialization
    # ChannelFactoryInitialize(0, "eth0")
    ChannelFactoryInitialize(0)
    robot = RobotAgent()
    robot.Init()

    try:
        print("Start moving.")
        robot.run_rectangle()
    except KeyboardInterrupt:
        print("Stop by Keyboard.")

