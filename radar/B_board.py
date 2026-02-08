import cv2
import numpy as np
import sys

# ===================== 配置参数 =====================
MARKER_LENGTH = 0.15  # 标记边长（米），必须与A_board.py一致
IMG_PATH_B = "cameraB.jpg"  # 相机B的图像
T_MARKER_TO_A_FILE = "T_marker_to_A.txt"  # A_board.py输出的矩阵文件

# ===================== 兼容不同OpenCV版本的工具函数 =====================
def create_aruco_detector_params():
    """兼容创建DetectorParameters"""
    # 优先使用 create 接口（部分版本提供 DetectorParameters_create）
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        try:
            return cv2.aruco.DetectorParameters_create()
        except Exception:
            pass
    # 有些版本可以直接实例化类
    if hasattr(cv2.aruco, "DetectorParameters"):
        try:
            return cv2.aruco.DetectorParameters()
        except Exception:
            pass
    return None

def get_aruco_dictionary(dict_type=cv2.aruco.DICT_6X6_250):
    """兼容获取Aruco字典"""
    try:
        return cv2.aruco.getPredefinedDictionary(dict_type)
    except AttributeError:
        if hasattr(cv2.aruco, "Dictionary_get"):
            return cv2.aruco.Dictionary_get(dict_type)
        raise


def detect_markers(image, dict_obj, params):
    """
    兼容检测marker：优先使用 cv2.aruco.detectMarkers；若不存在则用 ArucoDetector
    返回 (corners, ids, rejected)
    """
    # 老接口存在
    if hasattr(cv2.aruco, "detectMarkers"):
        return cv2.aruco.detectMarkers(image, dict_obj, parameters=params)
    # 新接口：ArucoDetector
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dict_obj, params)
        return detector.detectMarkers(image)
    raise AttributeError("cv2.aruco has no detectMarkers or ArucoDetector")


def estimate_pose_single_markers(corners_list, markerLen, cameraMatrix, distCoeffs):
    """
    兼容估计姿态：优先使用 aruco.estimatePoseSingleMarkers，
    若不可用则对每个标记用 solvePnP 估计。
    返回 rvecs, tvecs, _ （与原函数接口兼容）
    """
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        return cv2.aruco.estimatePoseSingleMarkers(corners_list, markerLen, cameraMatrix, distCoeffs)
    objp = np.array([
        [-markerLen/2,  markerLen/2, 0.0],
        [ markerLen/2,  markerLen/2, 0.0],
        [ markerLen/2, -markerLen/2, 0.0],
        [-markerLen/2, -markerLen/2, 0.0],
    ], dtype=np.float64)
    rvecs = []
    tvecs = []
    for corners in corners_list:
        pts = np.asarray(corners).reshape(-1, 2).astype(np.float64)
        success, rvec, tvec = cv2.solvePnP(objp, pts, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            rvecs.append(np.zeros(3, dtype=np.float64))
            tvecs.append(np.zeros(3, dtype=np.float64))
        else:
            rvecs.append(rvec.flatten())
            tvecs.append(tvec.flatten())
    return np.array(rvecs), np.array(tvecs), None

# ===================== 核心功能函数 =====================
def initCameraParams():
    """初始化相机B的内参和畸变系数（替换为实际标定值！）"""
    cameraMatrix = np.array([
        [1.813667282570175e+03,0,7.154124980116547e+02],
        [0,1.813670024456221e+03,5.565628860023487e+02],
        [0, 0, 1]
    ], dtype=np.float64)
    #distCoeffs = np.zeros((1, 5), dtype=np.float64)
    distCoeffs = np.array([[-0.076422298629092,0.152040702223770, 0, 0, 0]], dtype=np.float64)

    return cameraMatrix, distCoeffs

def detectSingleMarker(img, dict_obj, params):
    """安全检测单标记"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    corners, ids, rejected = detect_markers(gray, dict_obj, params)
    
    if ids is None or len(ids) != 1 or len(corners) != 1:
        print(f"错误：检测到{len(ids) if ids is not None else 0}个标记（单标记场景需仅1个）！")
        return False, [], -1
    
    markerCorners = corners[0]
    markerId = ids[0][0]
    return True, markerCorners, markerId

def estimateSingleMarkerPose(corners, markerLen, cameraMatrix, distCoeffs):
    """安全估计单标记姿态"""
    try:
        rvecs, tvecs, _ = estimate_pose_single_markers([corners], markerLen, cameraMatrix, distCoeffs)
        if rvecs is None or tvecs is None or len(rvecs) == 0 or len(tvecs) == 0:
            print("错误：姿态估计返回空的旋转/平移向量！")
            return False, None, None
        return True, rvecs[0].astype(np.float64), tvecs[0].astype(np.float64)
    except Exception as e:
        print(f"姿态估计失败：{e}")
        return False, None, None

def load_transform_matrix(filename):
    """从文件加载变换矩阵"""
    try:
        data = np.loadtxt(filename)
        if data.shape == (4, 4):
            return data
        elif data.shape == (16,):  # 展平的矩阵
            return data.reshape(4, 4)
        else:
            print(f"错误：文件格式不正确，期望4x4矩阵，得到{data.shape}")
            return None
    except Exception as e:
        print(f"加载矩阵文件失败：{e}")
        return None

def build_transform_matrix(R, t):
    """构建4x4变换矩阵"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def rotation_matrix_to_euler_angles(R):
    """从旋转矩阵计算欧拉角（ZYX顺序，最常见）"""
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])  # roll (绕X轴)
        y = np.arctan2(-R[2, 0], sy)      # pitch (绕Y轴)
        z = np.arctan2(R[1, 0], R[0, 0])  # yaw (绕Z轴)
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
        
    return np.array([x, y, z])  # roll, pitch, yaw

def rotation_matrix_to_euler_angles_xyz(R):
    """从旋转矩阵计算欧拉角（XYZ顺序）"""
    # 检查gimbal lock
    if np.abs(R[0, 2]) != 1:
        y1 = -np.arcsin(R[0, 2])  # pitch
        y2 = np.pi - y1
        x1 = np.arctan2(R[1, 2]/np.cos(y1), R[2, 2]/np.cos(y1))  # roll
        x2 = np.arctan2(R[1, 2]/np.cos(y2), R[2, 2]/np.cos(y2))
        z1 = np.arctan2(R[0, 1]/np.cos(y1), R[0, 0]/np.cos(y1))  # yaw
        z2 = np.arctan2(R[0, 1]/np.cos(y2), R[0, 0]/np.cos(y2))
        
        # 选择合理的解
        if np.abs(x1) + np.abs(y1) + np.abs(z1) <= np.abs(x2) + np.abs(y2) + np.abs(z2):
            return np.array([x1, y1, z1])
        else:
            return np.array([x2, y2, z2])
    else:
        # Gimbal lock情况
        z = 0  # 可以任意值
        if R[0, 2] == -1:
            y = np.pi/2
            x = z + np.arctan2(R[1, 0], R[2, 0])
        else:
            y = -np.pi/2
            x = -z + np.arctan2(-R[1, 0], -R[2, 0])
        return np.array([x, y, z])

def print_euler_angles(euler_angles, angle_type="ZYX"):
    """打印欧拉角（弧度/角度）"""
    roll, pitch, yaw = euler_angles
    print(f"欧拉角({angle_type}顺序):")
    print(f"  弧度: roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")
    print(f"  角度: roll={roll*180/np.pi:.2f}°, pitch={pitch*180/np.pi:.2f}°, yaw={yaw*180/np.pi:.2f}°")

# ===================== 主函数 =====================
def main():
    # 1. 加载相机A的变换矩阵
    print(f"正在加载相机A的变换矩阵: {T_MARKER_TO_A_FILE}")
    T_marker_to_A = load_transform_matrix(T_MARKER_TO_A_FILE)
    if T_marker_to_A is None:
        print("错误：无法加载相机A的变换矩阵，请先运行A_board.py")
        return -1
    
    print(f"成功加载T_marker_to_A:\n{T_marker_to_A}")
    
    # 2. 初始化Aruco检测器
    dictionary = get_aruco_dictionary(cv2.aruco.DICT_6X6_250)
    parameters = create_aruco_detector_params()
    
    # 3. 初始化相机B参数
    cameraMatrix_B, distCoeffs_B = initCameraParams()
    
    # 4. 读取相机B图像
    imgB = cv2.imread(IMG_PATH_B)
    if imgB is None:
        print(f"错误：图像读取失败！路径: {IMG_PATH_B}")
        return -1
    
    # 5. 检测相机B中的标记
    success_B, cornersB, idB = detectSingleMarker(imgB, dictionary, parameters)
    if not success_B:
        print("相机B标记检测失败！")
        return -1
    
    print(f"相机B检测到的标记ID: {idB}")
    
    # 6. 估计相机B的姿态
    success_rvecB, rvecB, tvecB = estimateSingleMarkerPose(
        cornersB, MARKER_LENGTH, cameraMatrix_B, distCoeffs_B
    )
    if not success_rvecB:
        print("相机B姿态估计失败！")
        return -1
    
    # 7. 可视化
    imgB_draw = imgB.copy()
    #cv2.aruco.drawAxis(imgB_draw, cameraMatrix_B, distCoeffs_B, rvecB, tvecB, MARKER_LENGTH * 0.5)
    cv2.aruco.drawDetectedMarkers(imgB_draw, [cornersB], np.array([idB]))
    cv2.imshow("Camera B (Single Marker)", imgB_draw)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 8. 构建相机B的变换矩阵
    R_B, _ = cv2.Rodrigues(rvecB)
    T_marker_to_B = build_transform_matrix(R_B, tvecB)
    
    print("\n" + "="*50)
    print("相机B变换矩阵:")
    print(f"T_marker_to_B:\n{T_marker_to_B}")
    
    # 9. 计算相机A和B之间的变换
    # 9.1 计算marker到相机的逆矩阵
    T_A_to_marker = np.linalg.inv(T_marker_to_A)
    
    # 9.2 计算A到B的变换
    T_A_to_B = np.dot(T_marker_to_B, T_A_to_marker)
    
    # 9.3 计算B到A的变换
    T_B_to_A = np.linalg.inv(T_A_to_B)
    
    print("\n" + "="*50)
    print("相机间变换矩阵:")
    print(f"T_A_to_B (相机A → 相机B):\n{T_A_to_B}")
    print(f"\nT_B_to_A (相机B → 相机A):\n{T_B_to_A}")
    
    # 10. 提取并打印欧拉角
    # 10.1 提取旋转部分
    R_A_to_B = T_A_to_B[:3, :3]
    t_A_to_B = T_A_to_B[:3, 3]
    
    # 10.2 计算旋转向量
    rvec_A_to_B, _ = cv2.Rodrigues(R_A_to_B)
    
    print("\n" + "="*50)
    print("相机A → 相机B 相对位姿:")
    print(f"平移向量: {t_A_to_B.T}")
    print(f"旋转向量: {rvec_A_to_B.T}")
    
    # 10.3 计算欧拉角 (ZYX顺序 - 最常见)
    euler_zyx = rotation_matrix_to_euler_angles(R_A_to_B)
    print_euler_angles(euler_zyx, "ZYX")
    
    # 10.4 计算欧拉角 (XYZ顺序)
    euler_xyz = rotation_matrix_to_euler_angles_xyz(R_A_to_B)
    print_euler_angles(euler_xyz, "XYZ")
    
    # 11. 验证变换一致性
    print("\n" + "="*50)
    print("变换一致性验证:")
    
    # 11.1 验证点变换
    point_in_marker = np.array([0, 0, 0, 1], dtype=np.float64).reshape(4, 1)
    point_in_A = np.dot(T_marker_to_A, point_in_marker)
    point_in_B_via_transform = np.dot(T_A_to_B, point_in_A)
    point_in_B_direct = np.dot(T_marker_to_B, point_in_marker)
    
    print(f"Marker原点在相机A坐标系: {point_in_A[:3].T}")
    print(f"Marker原点在相机B坐标系(直接): {point_in_B_direct[:3].T}")
    print(f"Marker原点在相机B坐标系(通过A→B变换): {point_in_B_via_transform[:3].T}")
    
    # 11.2 计算变换误差
    norm_error = np.linalg.norm(point_in_B_via_transform - point_in_B_direct)
    print(f"变换误差: {norm_error:.6f}")
    
    if norm_error < 1e-6:
        print("✓ 变换验证通过！")
    else:
        print("⚠ 警告：变换验证误差较大（但可能在浮点精度范围内）")
    
    # 12. 保存结果到文件
    np.savetxt("T_marker_to_B.txt", T_marker_to_B)
    np.savetxt("T_A_to_B.txt", T_A_to_B)
    np.savetxt("T_B_to_A.txt", T_B_to_A)
    
    print("\n" + "="*50)
    print("结果已保存到文件:")
    print("  - T_marker_to_B.txt")
    print("  - T_A_to_B.txt")
    print("  - T_B_to_A.txt")
    
    return 0

# ===================== 执行入口 =====================
if __name__ == "__main__":
    print(f"当前OpenCV版本：{cv2.__version__}")
    print("程序开始执行...")
    sys.exit(main())