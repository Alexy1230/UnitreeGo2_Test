"The robot moves a straight line trajectory"

import time
import math
import numpy as np
import cvxpy as cp
import threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.utils.thread import RecurrentThread

class RobotAgent:
    def __init__(self):
        self.gamma = 5.0
        self.kp_ = 0.5
        self.kp_ang_ = 0.4
        self.lock = threading.Lock()

        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.data_received = False
        self.running = True

        # current velocity commands (written by logic, read by control thread)
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_vyaw = 0.0
        self._cmd_lock = threading.Lock()

        self.obs_r = 0.3
        self.robot_r = 0.5
        self.buffer = 0.2

    def Init(self):
        self.sportstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sportstate_subscriber.Init(self.SportStateHandler, 10)

        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        time.sleep(1.0)

        self.sport_client.RecoveryStand()
        time.sleep(3.0)

        self.control_thread = RecurrentThread(
            interval=0.02, target=self._control_loop, name="control"
        )
        self.control_thread.Start()

    def _control_loop(self):
        """Runs at 50Hz — always sends the latest command to the robot."""
        with self._cmd_lock:
            vx = self._cmd_vx
            vy = self._cmd_vy
            vyaw = self._cmd_vyaw
        self.sport_client.Move(vx, vy, vyaw)

    def _set_cmd(self, vx, vy, vyaw):
        with self._cmd_lock:
            self._cmd_vx = vx
            self._cmd_vy = vy
            self._cmd_vyaw = vyaw

    def _stop_cmd(self):
        self._set_cmd(0.0, 0.0, 0.0)

    def SportStateHandler(self, msg):
        with self.lock:
            p = msg.position
            v = msg.velocity
            yaw = msg.imu_state.rpy[2]
            self.position = [p[0], p[1], p[2]]
            self.velocity = [v[0], v[1], v[2]]
            self.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            self.data_received = True
    
    def cbf_qp(self, u_ref, obs_pos):
        """
        CBF-QP in world frame.
        u_ref in world frame
        obs_pos in world frame
        """
        with self.lock:
            current_x = self.position[0]
            current_y = self.position[1]

        # world frame distance
        dx = current_x - obs_pos[0]
        dy = current_y - obs_pos[1]

        h = dx**2 + dy**2 - (self.robot_r + self.obs_r + self.buffer)**2
        grad_h = 2 * np.array([dx, dy])  # world frame

        u = cp.Variable(2)
        objective = cp.Minimize(cp.sum_squares(u - u_ref))
        constraint = grad_h @ u + self.gamma * h >= 0
        prob = cp.Problem(objective, [constraint])
        prob.solve(solver=cp.OSQP, warm_start=True)

        if u.value is None:
            print("CBF QP infeasible, stopping!")
            return np.array([0, 0, 0])

        return u.value  # world frame


    def _run_obs_avoidance(self, timeout=30.0):
        dest = 0.3*10  # 2.1m destination
        goal_pos = np.array([dest, 0])

        # Obstacle setting
        obs_pos = np.array([0.3*5, 0])

        vx_max = 0.3
        vy_max = 0.3
        vyaw_max = 0.5
        start_time = time.time()
        self._debug_i = 0

        while self.running:
            if not self.data_received:
                time.sleep(0.05)
                continue

            with self.lock:
                position = self.position[:]
                yaw = self.yaw

            # world frame distance to goal
            dx_goal = goal_pos[0] - position[0]
            dy_goal = goal_pos[1] - position[1]
            dist_goal = math.hypot(dx_goal, dy_goal)

            if dist_goal < 0.30:
                print(f"Position reached (dist={dist_goal:.3f}m)")
                self._stop_cmd()
                time.sleep(0.5)
                self.running = False
                break

            if time.time() - start_time > timeout:
                print(f"Failed. Movement timeout!")
                self._stop_cmd()
                time.sleep(0.5)
                self.running = False
                break

            u_ref_world = np.zeros(2)
            u_ref_world[0] = max(-vx_max, min(vx_max, dx_goal * self.kp_))
            u_ref_world[1] = max(-vy_max, min(vy_max, dy_goal * self.kp_))

            # solve CBF-QP in world frame
            u_world = self.cbf_qp(u_ref=u_ref_world, obs_pos=obs_pos)

            # convert to body frame
            cos_y = math.cos(-yaw)
            sin_y = math.sin(-yaw)
            vx_body = u_world[0] * cos_y - u_world[1] * sin_y
            vy_body = u_world[0] * sin_y + u_world[1] * cos_y

            target_yaw = math.atan2(dy_goal, dx_goal)
            yaw_error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
            vyaw_cmd = max(-vyaw_max, min(vyaw_max, self.kp_ang_ * yaw_error))

            self._set_cmd(vx_body, vy_body, vyaw_cmd)

            self._debug_i += 1
            if self._debug_i % 25 == 0:
                print(f"dist={dist_goal:.3f}m, vx_body={vx_body:.3f}, vy_body={vy_body:.3f}, vyaw={vyaw_cmd:.3f}")

            time.sleep(0.02)

        self.sport_client.StopMove()
        time.sleep(0.5)
        print(f"Final position: ({self.position[0]:.2f}, {self.position[1]:.2f}, {math.degrees(self.yaw):.1f}deg)")


if __name__ == "__main__":
    ChannelFactoryInitialize(0, "eth0")
    robot = RobotAgent()
    robot.Init()

    try:
        input("Press Enter to start...")
        print("Waiting for state data...")
        while not robot.data_received:
            time.sleep(0.1)

        # robot.single_step()
        robot._run_obs_avoidance()

    except KeyboardInterrupt:
        print("Interrupted.")
        robot.running = False

    finally:
        robot._stop_cmd()
        time.sleep(0.5)
        robot.control_thread.Wait()
        robot.sport_client.StopMove()
        time.sleep(1.0)
        print("Shutdown complete.")