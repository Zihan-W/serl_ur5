import numpy as np

# 加载.npy文件
camera_matrix = np.load('serl_robot_infra/ur_env/camera/cam0_wrt_table.npy')
camera_matrix = np.load('serl_robot_infra/ur_env/camera/PointCloudFusionFinetuned.npy')

# 打印矩阵内容
print("相机内参矩阵:")
print(camera_matrix)