"The robot moves a straight line trajectory"

import time
import math
import threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.utils.thread import RecurrentThread

class RobotAgent:
    def __init__(self):
        self.kp_ = 0.5
        self.kp_ang_ = 0.4
        self.lock = threading.Lock()

        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vyaw = 0.0
        self.v_ramp = 0.05

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
    
    def _translate_to(self, target_x, target_y, timeout=30.0):
        """Move to target position, ignoring rotation."""
        vx_max = 0.3
        vy_max = 0.3
        self.last_vx, self.last_vy, self.last_vyaw = 0.0, 0.0, 0.0
        start_time = time.time()

        self._debug_i = 0
        while self.running:
            if not self.data_received:
                time.sleep(0.1)
                continue

            with self.lock:
                position = self.position[:]
                yaw = self.yaw

            world_error_x = target_x - position[0]
            world_error_y = target_y - position[1]

            cos_y = math.cos(-yaw)
            sin_y = math.sin(-yaw)
            body_error_x = world_error_x * cos_y - world_error_y * sin_y
            body_error_y = world_error_x * sin_y + world_error_y * cos_y

            error_dist = math.sqrt(body_error_x**2 + body_error_y**2)

            self._debug_i += 1
            if self._debug_i % 25 == 0:
                print(f"  [translate] dist={error_dist:.3f}m  vx={self.last_vx:.3f}  vy={self.last_vy:.3f}")

            if error_dist < 0.35:
                print(f"  Position reached (dist={error_dist:.3f}m)")
                self._stop_cmd()
                time.sleep(0.5)
                return True

            if time.time() - start_time > timeout:
                print(f"  Translate timeout!")
                self._stop_cmd()
                time.sleep(0.5)
                return False

            raw_vx = max(-vx_max, min(vx_max, body_error_x * self.kp_))
            raw_vy = max(-vy_max, min(vy_max, body_error_y * self.kp_))

            self.last_vx += max(-self.v_ramp, min(self.v_ramp, raw_vx - self.last_vx))
            self.last_vy += max(-self.v_ramp, min(self.v_ramp, raw_vy - self.last_vy))
            self.last_vyaw = 0.0  # ← no rotation during translation

            self._set_cmd(self.last_vx, self.last_vy, self.last_vyaw)
            time.sleep(0.02)

        return False


    def _rotate_to(self, target_yaw, timeout=15.0):
        """Rotate to target yaw, ignoring translation."""
        vyaw_max = 0.3
        vyaw_ramp = 0.02
        self.last_vx, self.last_vy, self.last_vyaw = 0.0, 0.0, 0.0
        start_time = time.time()

        self._debug_i = 0
        while self.running:
            if not self.data_received:
                time.sleep(0.1)
                continue

            with self.lock:
                yaw = self.yaw

            error_ang = math.atan2(
                math.sin(target_yaw - yaw),
                math.cos(target_yaw - yaw)
            )

            self._debug_i += 1
            if self._debug_i % 25 == 0:
                print(f"  [rotate] ang_err={math.degrees(error_ang):.1f}deg  vyaw={self.last_vyaw:.3f}")

            if abs(error_ang) < 0.12:  # ~5 degrees
                print(f"  Rotation reached (err={math.degrees(error_ang):.1f}deg)")
                self._stop_cmd()
                time.sleep(0.5)
                return True

            if time.time() - start_time > timeout:
                print(f"  Rotate timeout!")
                self._stop_cmd()
                time.sleep(0.5)
                return False

            raw_vyaw = max(-vyaw_max, min(vyaw_max, error_ang * self.kp_ang_))
            self.last_vyaw += max(-vyaw_ramp, min(vyaw_ramp, raw_vyaw - self.last_vyaw))
            self.last_vx = 0.05
            self.last_vy = 0.0

            self._set_cmd(self.last_vx, self.last_vy, self.last_vyaw)
            time.sleep(0.02)

        return False


    def run_rectangle(self):
        L = 0.3 * 4  # 1.2m sides

        with self.lock:
            start_x = self.position[0]
            start_y = self.position[1]
            start_yaw = self.yaw

        # Build 4 corners + 4 rotations = 8 steps
        steps = []
        x, y, a = start_x, start_y, start_yaw
        for _ in range(4):
            # next corner position
            x += L * math.cos(a)
            y += L * math.sin(a)
            a_next = math.atan2(math.sin(a + math.pi/2), math.cos(a + math.pi/2))
            steps.append(('translate', x, y))
            steps.append(('rotate', a_next))
            a = a_next

        print("Rectangle steps:")
        for i, s in enumerate(steps):
            if s[0] == 'translate':
                print(f"  {i+1}: move to ({s[1]:.2f}, {s[2]:.2f})")
            else:
                print(f"  {i+1}: rotate to {math.degrees(s[1]):.1f}deg")

        for i, step in enumerate(steps):
            if not self.running:
                break
            print(f"\nStep {i+1}/8: {step[0]}...")

            if step[0] == 'translate':
                success = self._translate_to(step[1], step[2])
            else:
                success = self._rotate_to(step[1])

            if not success:
                print("Step failed — aborting rectangle.")
                break
        else:
            print("\nRectangle complete!")

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
        robot.run_rectangle()

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