import numpy as np

# 加载.npy文件
camera_matrix = np.load('serl_robot_infra/ur_env/camera/cam0_wrt_table.npy')
camera_matrix = np.load('serl_robot_infra/ur_env/camera/PointCloudFusionFinetuned.npy')

# 打印矩阵内容
print("相机内参矩阵:")
print(camera_matrix)

import numpy as np
from scipy.spatial.transform import Rotation as R

# 给定的旋转矩阵
rotation_matrix = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.86162916, 0.50753836],
    [0.0, -0.50753836, 0.86162916]
])

# 计算欧拉角
r = R.from_matrix(rotation_matrix)
euler_angles = r.as_euler("xyz", degrees=True)

# 输出欧拉角
print("euler_angles",euler_angles)
