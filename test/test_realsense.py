import pyrealsense2 as rs
import numpy as np
import cv2
import time

def test_realsense():
    try:
        # 创建Pipeline对象
        pipeline = rs.pipeline()
        config = rs.config()

        # 获取设备信息
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            print("错误：未检测到RealSense相机!")
            return False
        
        print(f"检测到 {len(devices)} 个RealSense设备")
        for device in devices:
            print(f"设备名称: {device.get_info(rs.camera_info.name)}")
            print(f"序列号: {device.get_info(rs.camera_info.serial_number)}")
            print(f"固件版本: {device.get_info(rs.camera_info.firmware_version)}")
            print("-" * 50)

        # 配置数据流
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # 启动Pipeline
        print("正在启动数据流...")
        pipeline.start(config)
        print("数据流启动成功!")

        try:
            for i in range(1000):
                # 等待新的帧
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    print("未接收到帧，重试中...")
                    continue

                # 转换为numpy数组
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # 将深度图像转换为伪彩色图像以便显示
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )

                # 显示图像
                cv2.imshow('RealSense RGB', color_image)
                cv2.imshow('RealSense Depth', depth_colormap)

                # 按'q'退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("测试完成!")
                    break

        finally:
            # 停止Pipeline
            pipeline.stop()
            cv2.destroyAllWindows()
            print("相机已关闭")

    except Exception as e:
        print(f"发生错误: {str(e)}")
        return False

    return True

if __name__ == "__main__":
    print("开始RealSense相机测试...")
    print("按'q'键退出测试")
    test_realsense() 