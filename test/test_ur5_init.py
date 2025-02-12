import numpy as np
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

def test_ur5_control(robot_ip):
    # 创建UR控制接口
    ur_control = RTDEControlInterface(robot_ip)
    ur_receive = RTDEReceiveInterface(robot_ip)
    pos = ur_receive.getActualTCPPose()

    
    try:
        # 设置负载和TCP
        ur_control.setPayload(0.5, [0, 0, 0])
        ur_control.setTcp([0, 0, 0.23, 0, 0, np.pi-0.2617994])

        # target_pos = [-0.2317, -0.2508, 0.2944, -0.5032, 0.8638, 0.0245, 0.0009] # not safe
        target_pos = [-0.21, 0.3, 0.2, -0.001, 3.12, 0.04]

        # 初始关节角度
        tool_ini_joints = [0.5428823828697205, -1.9081628958331507, 1.6074042320251465,
                          -1.2644203344928187, -1.6236584822284144, 1.3247456550598145]
        
        # 运动参数
        acceleration = 0.8  # rad/s^2
        velocity = 0.4     # rad/s
        
        # ur_control.moveL(target_pos, velocity/2, acceleration/2)

        # 移动到初始位置
        ur_control.moveJ(tool_ini_joints, velocity, acceleration)
        
        print("机器人初始化成功!")
        print(pos)
        
    except Exception as e:
        print(f"初始化失败: {str(e)}")
        
    finally:
        # 断开连接
        ur_control.disconnect()
        
def test_ur5_receive(robot_ip):
    rtde_r = RTDEReceiveInterface(robot_ip)
    actual_q = rtde_r.getActualQ()
    actual_tcp_pose = rtde_r.getActualTCPPose()
    print("actual_tcp_pose",actual_tcp_pose)
    roll = np.arctan2(2 * (actual_tcp_pose[5] * actual_tcp_pose[4] + actual_tcp_pose[3] * actual_tcp_pose[6]), 1 - 2 * (actual_tcp_pose[4]**2 + actual_tcp_pose[5]**2))
    pitch = np.arcsin(2 * (actual_tcp_pose[5] * actual_tcp_pose[4] - actual_tcp_pose[6] * actual_tcp_pose[3]))
    yaw = np.arctan2(2 * (actual_tcp_pose[5] * actual_tcp_pose[6] + actual_tcp_pose[3] * actual_tcp_pose[4]), 1 - 2 * (actual_tcp_pose[4]**2 + actual_tcp_pose[6]**2))
    actual_tcp_pose[3:] = [roll, pitch, yaw]  # 更新为rpy
    print("actual_tcp_pose (rpy)", actual_tcp_pose)

    # roll, pitch, yaw = actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]
    # qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    # qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    # qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    # qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    # actual_tcp_pose[3:] = [qx, qy, qz, qw]  # 更新为四元数
    # print("actual_q",actual_q)
    # print("actual_tcp_pose",actual_tcp_pose)
    # # 将四元数转换为滚转、俯仰和偏航角（rpy）
    # qx, qy, qz, qw = actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5], actual_tcp_pose[6]
    # # 计算滚转（roll）
    # roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    # # 计算俯仰（pitch）
    # pitch = np.arcsin(2 * (qw * qy - qz * qx))
    # # 计算偏航（yaw）
    # yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

    # actual_tcp_pose[3:] = [roll, pitch, yaw]  # 更新为rpy
    # print("actual_tcp_pose (rpy)", actual_tcp_pose)

if __name__ == "__main__":
    robot_ip = "192.168.1.101"
    test_ur5_control(robot_ip)
    test_ur5_receive(robot_ip)