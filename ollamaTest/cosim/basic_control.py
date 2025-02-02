# 장애물 감지

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard
from pynput.keyboard import Key, Listener
import time
import numpy as np


class PioneerP3DXController:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.run_flag = True
        self.auto_mode = False

        try:
            self.left_motor = self.sim.getObject("/PioneerP3DX/leftMotor")
            self.right_motor = self.sim.getObject("/PioneerP3DX/rightMotor")
            self.sensors = [
                self.sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]")
                for i in range(16)
            ]
        except Exception as e:
            print("초기화 오류:", e)
            self.run_flag = False
            return

        self.movement_state = "STOP"  # STOP, FORWARD, BACKWARD
        self.base_speed = 2.0
        self.OBSTACLE_THRESHOLD = 0.5
        self.CRITICAL_THRESHOLD = 0.3
        self.listener = None

    def get_sensor_data(self):
        distances = []
        for sensor in self.sensors:
            _, state, distance_data, _, _ = self.sim.readProximitySensor(sensor)
            if state and distance_data:
                distance = (
                    distance_data[2]
                    if isinstance(distance_data, list)
                    else distance_data
                )
                distances.append(distance)
            else:
                distances.append(float("inf"))
        return distances

    def analyze_environment(self, distances):
        front_distances = distances[3:7]  # 전방 센서
        left_distances = distances[0:3]  # 좌측 센서
        right_distances = distances[7:10]  # 우측 센서
        back_distances = distances[12:15]  # 후방 센서

        min_front = min(front_distances)
        min_left = min(left_distances)
        min_right = min(right_distances)
        min_back = min(back_distances)

        if min_front < self.CRITICAL_THRESHOLD:
            if min_back > self.OBSTACLE_THRESHOLD:
                return "BACKWARD"
            else:
                left_space = sum(left_distances) / len(left_distances)
                right_space = sum(right_distances) / len(right_distances)
                return "TURN_LEFT" if left_space > right_space else "TURN_RIGHT"

        if min_front > self.OBSTACLE_THRESHOLD:
            return "FORWARD"

        # 장애물 회피 방향 결정
        left_space = sum(left_distances) / len(left_distances)
        right_space = sum(right_distances) / len(right_distances)
        return "TURN_LEFT" if left_space > right_space else "TURN_RIGHT"

    def control_movement(self, action):
        left_vel = right_vel = 0

        if action == "FORWARD":
            left_vel = right_vel = self.base_speed
        elif action == "BACKWARD":
            left_vel = right_vel = -self.base_speed
        elif action == "TURN_LEFT":
            left_vel = -self.base_speed / 2
            right_vel = self.base_speed / 2
        elif action == "TURN_RIGHT":
            left_vel = self.base_speed / 2
            right_vel = -self.base_speed / 2

        self.sim.setJointTargetVelocity(self.left_motor, left_vel)
        self.sim.setJointTargetVelocity(self.right_motor, right_vel)

    def on_press(self, key):
        if key == Key.space:
            self.auto_mode = not self.auto_mode
            print(f"자율주행 모드: {'켜짐' if self.auto_mode else '꺼짐'}")
        elif key == Key.esc:
            self.stop()
            return False

    def stop(self):
        self.run_flag = False
        if self.listener:
            self.listener.stop()
        try:
            self.sim.stopSimulation()
        except:
            pass

    def run_simulation(self):
        try:
            self.sim.startSimulation()
            with Listener(on_press=self.on_press) as listener:
                self.listener = listener
                while self.run_flag:
                    if self.auto_mode:
                        distances = self.get_sensor_data()
                        action = self.analyze_environment(distances)
                        self.control_movement(action)
                    else:
                        self.control_movement("STOP")
                    time.sleep(0.1)
        except Exception as e:
            print("시뮬레이션 오류:", e)
        finally:
            self.stop()


if __name__ == "__main__":
    pioneer_controller = PioneerP3DXController()
    pioneer_controller.run_simulation()
