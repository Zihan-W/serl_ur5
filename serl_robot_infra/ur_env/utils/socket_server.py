import socket
import json
import time
import numpy as np
import threading

class SocketServer:
    def __init__(self, start_pose, ip_port=('169.254.91.200', 34566)):
        self.state_lock = threading.Lock()
        self.ip_port = ip_port
        self.server, self.sk, self.addr = self.setup()
        self.running = True
        self.latest_data = {"action": np.zeros(14), "button_x": 0, "button_a": 0}  # 用于存储最新的动作数据
        self.thread = threading.Thread(target=self.socket_to_action, args=(start_pose,))
        self.thread.start()

    def setup(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(self.ip_port)
        server.listen(1)
        sk, addr = server.accept()
        return server, sk, addr

    def request(self):
        req = "1"
        self.sk.send(req.encode())

    def receive(self):
        data_ = self.sk.recv(1024)
        data = json.loads(data_)
        return data

    def send(self, data):
        data = json.dumps(data)
        self.sk.send(data.encode())

    def end(self):
        req = '0'
        self.sk.send(req.encode())
        self.running = False
        self.server.close()

    def socket_to_action(self, start_pose):
        offset = self.calibrate_dual_arx(start_pose)  # 校准
        actions = []  # 用于保存动作数据
        print("Quest3 start_pose:", start_pose, "offset:", offset)

        while self.running:  # 遍历时间步
            t0 = time.time()
            # 从 socket 获取数据并转换为动作
            action, button_x, button_a = self.get_master_action_arx(offset)

            # 更新最新的动作数据
            self.latest_data["action"] = action
            self.latest_data["button_x"] = button_x
            self.latest_data["button_a"] = button_a

            # 输出动作和相关信息
            # print(f"Socket_Server, Action: {action[3:6]}, Button_X: {button_x}, Button_A: {button_a}")

            actions.append(action)  # 保存动作数据

            # 如果按下 Button_X，则中断
            if button_x == 1:
                print("Button X pressed, stopping...")
                self.end()
                break

            # 控制循环的时间步间隔
            time.sleep(max(0, (1 / 10) - (time.time() - t0)))  # 假设目标 FPS 为 10

        print("Data collection completed.")

    def get_master_action_arx(self, offset):
        self.request()
        action_ = self.receive()
        action, button_x, button_a = self.master_to_arm_arx(action_, offset)
        return action, button_x, button_a

    def master_to_arm_arx(self, action, offset):
        roll = -action[2]
        pitch = -action[0]
        yaw = action[1]
        x = -action[5]
        y = -action[3]
        z = action[4]
        gripper = action[6] # 0-1, 0: 按下按键, 1: 松开按键

        roll1 = -action[9]
        pitch1 = -action[7]
        yaw1 = action[8]
        x1 = -action[12]
        y1 = -action[10]
        z1 = action[11]
        gripper1 = action[13]
        master_action = np.array([roll, pitch, yaw, x, y, z, 4.5 - 4.5 * gripper,
                                  roll1, pitch1, yaw1, x1, y1, z1, 4.5 - 4.5 * gripper1])

        button_x = action[16]
        button_a = action[14]
        return master_action + offset, button_x, button_a

    def calibrate_dual_arx(self, start_pose=None):
        print("start calibrating,don't move the controller!")
        count = np.zeros(14)
        for i in range(50):
            self.request()
            action_ = self.receive()
            action_ = np.array(action_)
            count += action_[:14]
        print("calibration done")
        count = count / 50

        roll1 = -count[2]
        pitch1 = -count[0]
        yaw1 = count[1]
        x1 = -count[5]
        y1 = -count[3]
        z1 = count[4]

        roll2 = -count[9]
        pitch2 = -count[7]
        yaw2 = count[8]
        x2 = -count[12]
        y2 = -count[10]
        z2 = count[11]

        if start_pose is None:
            return np.concatenate([
                np.array([-roll1, -pitch1, -yaw1, 0. - x1, -y1, 0. - z1, 0.]),
                np.array([-roll2, -pitch2, -yaw2, 0. - x2, -y2, 0. - z2, 0.])
            ])
        else:
            return np.concatenate([
                np.array([-roll1, -pitch1, -yaw1, -x1, -y1, -z1, 0.]) + np.array([start_pose[3], start_pose[4], start_pose[5], start_pose[0], start_pose[1], start_pose[2], 0.]),
                np.array([-roll2, -pitch2, -yaw2, 0. - x2, -y2, 0. - z2, 0.]) + np.zeros(7)
            ])

    def get_action(self):
        """返回最新的动作数据"""
        with self.state_lock:
            return self.latest_data["action"], self.latest_data["button_x"], self.latest_data["button_a"]

if __name__ == "__main__":
    server = SocketServer(ip_port=('169.254.91.200', 34566))
    server.thread.join()  # 等待线程结束
    # test_image()