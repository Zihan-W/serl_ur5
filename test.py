import numpy as np
from rtde_control import RTDEControlInterface

def test_ur5_ini():
    # 创建UR控制接口
    ur_control = RTDEControlInterface("192.168.1.101")
    
    try:
        # 设置负载和TCP
        ur_control.setPayload(0.5, [0, 0, 0])
        ur_control.setTcp([0, 0, 0.23, 0, 0, np.pi-0.2617994])

        # 初始关节角度
        tool_ini_joints = [0.5428823828697205, -1.9081628958331507, 1.6074042320251465,
                          -1.2644203344928187, -1.6236584822284144, 1.3247456550598145]
        
        # 运动参数
        acceleration = 0.8  # rad/s^2 
        velocity = 0.4     # rad/s
        
        # 移动到初始位置
        ur_control.moveJ(tool_ini_joints, velocity, acceleration)
        
        print("机器人初始化成功!")
        
    except Exception as e:
        print(f"初始化失败: {str(e)}")
        
    finally:
        # 断开连接
        ur_control.disconnect()
        
if __name__ == "__main__":
    test_ur5_ini()
