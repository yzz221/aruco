import cv2
import numpy as np
import sys

#需要修改1、标记边长  2、内参矩阵  3、欧拉角顺序ZYX/XYZ？

# ===================== 配置参数（与C++对应，按需修改） =====================
MARKER_LENGTH = 0.15  # 标记边长（米）
IMG_PATH_A = "cameraA.jpg"

# ===================== 兼容不同OpenCV版本的工具函数 =====================
def create_aruco_detector_params():
    """兼容创建DetectorParameters，适配OpenCV 3.x/4.x各版本"""
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
    # 兜底返回 None（调用方需能接受 None）
    return None

def get_aruco_dictionary(dict_type=cv2.aruco.DICT_6X6_250):
    """兼容获取Aruco字典（部分版本需特殊处理）"""
    try:
        return cv2.aruco.getPredefinedDictionary(dict_type)
    except AttributeError:
        # 兼容极旧版本写法
        if hasattr(cv2.aruco, "Dictionary_get"):
            return cv2.aruco.Dictionary_get(dict_type)
        # 有些新版本直接有 Dictionary 或 getPredefinedDictionary 可用的变体
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
        try:
            detector = cv2.aruco.ArucoDetector(dict_obj, params)
            return detector.detectMarkers(image)
        except Exception as e:
            raise
    raise AttributeError("cv2.aruco has no detectMarkers or ArucoDetector")


def estimate_pose_single_markers(corners_list, markerLen, cameraMatrix, distCoeffs):
    """
    兼容估计姿态：优先使用 aruco.estimatePoseSingleMarkers，
    若不可用则对每个标记用 solvePnP 估计。
    返回 rvecs, tvecs, _ （与原函数接口兼容）
    """
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        return cv2.aruco.estimatePoseSingleMarkers(corners_list, markerLen, cameraMatrix, distCoeffs)
    # 使用 solvePnP 作为替代实现（假定标记在 z=0 平面，四角顺序与 detectMarkers 输出一致）
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
    # 返回格式与 estimatePoseSingleMarkers 类似：rvecs, tvecs, _
    return np.array(rvecs), np.array(tvecs), None

# ===================== 核心功能函数 =====================
def initCameraParams():
    """初始化相机内参和畸变系数（替换为实际标定值！）"""
    cameraMatrix = np.array([
        [1.813667282570175e+03,0,7.154124980116547e+02],
        [0,1.813670024456221e+03,5.565628860023487e+02],
        [0, 0, 1]
    ], dtype=np.float64)
    #distCoeffs = np.zeros((1, 5), dtype=np.float64)  # 畸变系数（示例值）
    distCoeffs = np.array([[-0.076422298629092,0.152040702223770, 0, 0, 0]], dtype=np.float64)
    return cameraMatrix, distCoeffs

def detectSingleMarker(img, dict_obj, params):
    """
    安全检测单标记（对应C++的detectSingleMarker）
    返回：success, markerCorners, markerId
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    corners, ids, rejected = detect_markers(gray, dict_obj, params)
    
    # 单标记校验：必须仅检测到1个，且角点与ID长度匹配
    if ids is None or len(ids) != 1 or len(corners) != 1:
        print(f"错误：检测到{len(ids) if ids is not None else 0}个标记（单标记场景需仅1个）！")
        return False, [], -1
    
    markerCorners = corners[0]
    markerId = ids[0][0]
    return True, markerCorners, markerId

def estimateSingleMarkerPose(corners, markerLen, cameraMatrix, distCoeffs):
    """
    安全估计单标记姿态（对应C++的estimateSingleMarkerPose）
    返回：success, rvec, tvec
    """
    try:
        rvecs, tvecs, _ = estimate_pose_single_markers([corners], markerLen, cameraMatrix, distCoeffs)
        if rvecs is None or tvecs is None or len(rvecs) == 0 or len(tvecs) == 0:
            print("错误：姿态估计返回空的旋转/平移向量！")
            return False, None, None
        # 返回为 (N,3) 形式，提取第一个
        return True, rvecs[0].astype(np.float64), tvecs[0].astype(np.float64)
    except Exception as e:
        print(f"姿态估计失败：{e}")
        return False, None, None

def build_transform_matrix(R, t):
    """构建4x4变换矩阵（对应T_marker_to_A）"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def euler_from_rotation_matrix(R):
    """从旋转矩阵计算欧拉角（roll/pitch/yaw）"""
    sy = np.sqrt(R[0, 0] **2 + R[1, 0]** 2)
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw

# ===================== 主函数 =====================
def main():
    # 1. 初始化Aruco检测器（兼容版本）
    dictionary = get_aruco_dictionary(cv2.aruco.DICT_6X6_250)
    parameters = create_aruco_detector_params()

    # 2. 初始化相机参数
    cameraMatrix_A, distCoeffs_A = initCameraParams()

    # 3. 读取图像
    imgA = cv2.imread(IMG_PATH_A)
    if imgA is None:
        print("Error: 图像读取失败！")
        return -1
    
    # 4. 检测单标记
    success_A, cornersA, idA = detectSingleMarker(imgA, dictionary, parameters)
    if not success_A:
        print("相机A标记检测失败！")
        return -1
    
    # 5. 估计姿态
    success_rvecA, rvecA, tvecA = estimateSingleMarkerPose(
        cornersA, MARKER_LENGTH, cameraMatrix_A, distCoeffs_A
    )
    if not success_rvecA:
        print("相机A姿态估计失败！")
        return -1
    
    # 6. 给可视化留个位置

    # 7. 构建变换矩阵
    # 旋转向量转旋转矩阵
    R_A, _ = cv2.Rodrigues(rvecA)

    # 构建marker→相机的变换矩阵
    T_marker_to_A = build_transform_matrix(R_A, tvecA)

    #输出结果，这里应该发送出去
    print("T_marker_to_A:\n", T_marker_to_A)

    # 保存矩阵到文件
    np.savetxt("T_marker_to_A.txt", T_marker_to_A)
    print(f"变换矩阵已保存到: T_marker_to_A.txt")

    return 0

# ===================== 执行入口 =====================
if __name__ == "__main__":
    # 检查OpenCV版本（可选，用于调试）
    print(f"当前OpenCV版本：{cv2.__version__}")
    # 执行主函数
    sys.exit(main())