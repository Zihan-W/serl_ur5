import serial  # 导入串口通信库
from time import sleep

class SerialGripper:
    def __init__(self):
        self.ser = serial.Serial()
        # 十六进制数据给串口
        self.gripper_init = "0106010000A5484D"
        # self.gripper_init = "01060100000149F6"
        # self.gripper_close = "0106010301F47821"
        self.current_angle_pct = 1.0  # 记录当前的angle_pct,初始化为1.0(完全张开)

    def port_open_recv(self, init=0):  # 对串口的参数进行配置
        # self.ser.port = 'com0'
        self.ser.port = '/dev/ttyUSB0'
        self.ser.baudrate = 115200
        self.ser.bytesize = 8
        self.ser.stopbits = 1
        self.ser.parity = "N"  # 奇偶校验位
        if not (self.ser.isOpen()):
            self.ser.open()
        if (self.ser.isOpen()):
            print("串口打开成功！")
            if init==1:
                self.send_hex_data(self.gripper_init)
            # sleep(3)
        else:
            print("串口关闭，尝试打开！")
            self.ser.open()

    def port_close(self):
        self.ser.close()
        if (self.ser.isOpen()):
            print("串口关闭失败！")
        else:
            print("串口关闭成功！")

    # CRC校验
    def crc16(self, data):
        if isinstance(data, int):
            data = data.to_bytes((data.bit_length() + 7) // 8, 'big')
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def send_hex_data(self, hex_data):
        # hex_data="7b01020120492000c8f87d"
        # print(hex_data)
        hex_data_bytes = bytes.fromhex(hex_data)  # 将十六进制字符串转换为字节数据
        self.ser.write(hex_data_bytes)  # 发送字节数据到串口
        # print("发送成功:", hex_data_bytes)

    def get_current_angle_pct(self):
        """返回当前夹爪的开合程度百分比"""
        return self.current_angle_pct

    def gripper_control(self, angle_pct=1.0):
        if(self.ser.isOpen()):
            pass
        else:
            self.port_open_recv()

        if not 0 <= angle_pct <= 1 :
            raise ValueError("angle_pct 必须在 0 到 1 之间")
            
        # 更新当前angle_pct
        self.current_angle_pct = angle_pct
        
        #hex为16进制字符串
        head_hex= '01060103'
        angle_hex=hex(int(angle_pct*1000))[2:]
        angle_hex = angle_hex.zfill(4)
        first_string=head_hex+angle_hex
        # print(first_string)
        hex_representation = f"0x{first_string}"
        # print(type(hex_representation))
        hex_representation_int=int(hex_representation,16)
        # print(hex_representation_int)
        end_hex=hex(self.crc16(hex_representation_int))[2:].zfill(4)
        end_hex = ''.join([end_hex[i:i+2] for i in range(0, len(end_hex), 2)][::-1])
        # print(end_hex)
        gripper_action=head_hex+angle_hex+end_hex
        # print(gripper_action)
        self.send_hex_data(gripper_action)

    def gripper_open(self):
        """完全张开夹爪"""
        self.gripper_control(1.0)

    def gripper_close(self):
        """完全关闭夹爪"""
        self.gripper_control(0.0)


if __name__ == '__main__':
    gripper = SerialGripper()
    gripper.port_open_recv(1)

    ###完全张开
    # gripper.gripper_control(1)
    ###完全关闭
    # gripper.gripper_control(0)