from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard
from pynput.keyboard import Key, Listener
import time

# coppeliasim에서 방향키로만 조정해서 테스트 하기 - 단순형

class PioneerP3DXController:
    def __init__(self):
        # CoppeliaSim 연결 초기화
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.run_flag = True

        try:
            # 로봇의 왼쪽/오른쪽 모터와 초음파 센서 객체 가져오기
            self.left_motor = self.sim.getObject("/PioneerP3DX/leftMotor")
            self.right_motor = self.sim.getObject("/PioneerP3DX/rightMotor")
            self.sensors = [
                self.sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]")
                for i in range(16)
            ]
        except Exception as e:
            print("객체 초기화 오류:", e)
            self.run_flag = False
            return

        # 로봇 제어 변수 초기화
        self.vel_X = 0  # 전진/후진 속도
        self.vel_Y = 0  # 회전 속도
        self.delta_velocity = 1.0  # 속도 증가량
        self.turn_velocity = 0.5  # 회전 속도값
        self.listener = None

    def control_car(self):
        try:
            if self.run_flag:
                # 좌우 모터에 속도 설정 (회전을 위해 속도차 적용)
                self.sim.setJointTargetVelocity(
                    self.left_motor, self.vel_X - self.vel_Y
                )
                self.sim.setJointTargetVelocity(
                    self.right_motor, self.vel_X + self.vel_Y
                )
        except Exception as e:
            print("제어 오류:", e)
            self.stop()

    def read_sensors(self):
        try:
            if self.run_flag:
                # 모든 초음파 센서 값 읽기
                for i, sensor in enumerate(self.sensors):
                    _, state, distance_data, _, _ = self.sim.readProximitySensor(sensor)
                    if state and distance_data:
                        distance = (
                            distance_data[2]
                            if isinstance(distance_data, list)
                            else distance_data
                        )
                        print(f"센서 {i}: {distance:.2f}m")
        except Exception as e:
            print("센서 오류:", e)
            self.stop()

    def stop(self):
        # 안전한 종료를 위한 정리 작업
        self.run_flag = False
        if self.listener:
            self.listener.stop()
        try:
            self.sim.stopSimulation()
        except:
            pass

    def on_press(self, key):
        # 키보드 입력에 따른 속도 제어
        if not self.run_flag:
            return False
        try:
            if key == Key.up:
                self.vel_X = self.delta_velocity  # 전진
            elif key == Key.down:
                self.vel_X = -self.delta_velocity  # 후진
            elif key == Key.left:
                self.vel_Y = self.turn_velocity  # 좌회전
            elif key == Key.right:
                self.vel_Y = -self.turn_velocity  # 우회전
            elif key == Key.esc:
                self.stop()
                return False
        except Exception as e:
            print("키 입력 오류:", e)
            self.stop()
            return False

    def on_release(self, key):
        # 키를 뗐을 때의 속도 초기화
        if not self.run_flag:
            return False
        try:
            if key in [Key.up, Key.down]:
                self.vel_X = 0  # 전진/후진 정지
            elif key in [Key.left, Key.right]:
                self.vel_Y = 0  # 회전 정지
        except Exception as e:
            print("키 해제 오류:", e)
            self.stop()
            return False

    def run_simulation(self):
        try:
            # 시뮬레이션 시작 및 메인 루프
            self.sim.startSimulation()
            with Listener(
                on_press=self.on_press, on_release=self.on_release
            ) as listener:
                self.listener = listener
                while self.run_flag:
                    self.read_sensors()  # 센서값 읽기
                    self.control_car()  # 로봇 제어
                    self.sim.step()  # 시뮬레이션 한 스텝 진행
                    time.sleep(0.1)  # 적절한 실행 주기 유지
        except Exception as e:
            print("시뮬레이션 오류:", e)
        finally:
            self.stop()


if __name__ == "__main__":
    pioneer_controller = PioneerP3DXController()
    pioneer_controller.run_simulation()
