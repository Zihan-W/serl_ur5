import socket
import json
import time
import numpy as np

class SocketServer:
    def __init__(self, ip_port=('169.254.91.200', 34566)):
        self.ip_port = ip_port
        self.server, self.sk, self.addr = self.setup()

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
        self.server.close()

    def socket_to_action(self):
        offset = self.calibrate_dual_arx()  # 校准
        actions = []  # 用于保存动作数据
        max_timesteps = 1000  # 最大时间步数

        while True:  # 遍历时间步
            t0 = time.time()
            # 从 socket 获取数据并转换为动作
            action, button_x, button_a = self.get_master_action_arx(offset)

            # 输出动作和相关信息
            print(f", Action: {action}, Button_X: {button_x}, Button_A: {button_a}")

            actions.append(action)  # 保存动作数据

            # 如果按下 Button_X，则中断
            if button_x == 1:
                print("Button X pressed, stopping...")
                break

            # 控制循环的时间步间隔
            time.sleep(max(0, (1 / 60) - (time.time() - t0)))  # 假设目标 FPS 为 60

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
        gripper = action[6]

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
            return np.array([-roll1, -pitch1, -yaw1, 0. - x1, -y1, 0. - z1, 0.,
                             -roll2, -pitch2, -yaw2, 0. - x2, -y2, 0. - z2, 0.])
        else:
            return np.array([-roll1, -pitch1, -yaw1, - x1, -y1, - z1, 0.,
                             -roll2, -pitch2, -yaw2, - x2, -y2, - z2, 0.]) + start_pose

if __name__ == "__main__":
    server = SocketServer(ip_port=('169.254.91.200', 34566))
    server.socket_to_action()
    # test_image()