#!/usr/bin/env python3
"""
core2_5ros - 简化测试版本

功能：
1. 订阅 pB (敌方在雷达系的位置)
2. 订阅 T_R_At (我方相机在雷达系的位姿)
3. 计算投影到图像像素
4. 发布像素坐标

注意：只同步2个话题，移除了不必要的pA
"""
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Float32MultiArray

# message_filters for time synchronization
from message_filters import Subscriber as MFSubscriber
from message_filters import ApproximateTimeSynchronizer


# ========== 辅助函数 ==========
def make_T(R, t):
    """构建4x4变换矩阵"""
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t.flatten()
    return T


def quaternion_to_R(qx, qy, qz, qw):
    """四元数转旋转矩阵"""
    x, y, z, w = qx, qy, qz, qw
    n = x*x + y*y + z*z + w*w
    if n == 0.0:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    
    R = np.array([
        [1.0 - (yy + zz),  xy - wz,          xz + wy],
        [xy + wz,          1.0 - (xx + zz),  yz - wx],
        [xz - wy,          yz + wx,          1.0 - (xx + yy)]
    ])
    return R


# ========== 相机参数 ==========
# TODO: 替换为你的实际相机内参！
# 这些是你 A_board.py 中使用的内参
fx = 1.813667282570175e+03
fy = 1.813670024456221e+03
cx = 7.154124980116547e+02
cy = 5.565628860023487e+02

K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

# 畸变参数
dist = np.array([[-0.076422298629092,0.152040702223770, 0, 0, 0]], dtype=np.float64)


# ========== 投影函数 ==========
def project_B_to_image(pB_R, T_R_At):
    """
    将敌方从雷达系投影到我方相机图像
    
    参数:
        pB_R: 敌方在雷达系的位置，shape (3,)
        T_R_At: 我方相机在雷达系的完整位姿，4x4 矩阵
    
    返回:
        (pixel, pB_cam) 或 None
        pixel: (u, v) 像素坐标
        pB_cam: 敌方在相机系的3D坐标
    """
    T_R_At = np.asarray(T_R_At)
    if T_R_At.shape != (4,4):
        raise ValueError("T_R_At must be 4x4")
    
    # 计算 T_At_R (雷达系 → 相机系)
    T_At_R = np.linalg.inv(T_R_At)
    
    # 将敌方位置从雷达系变换到相机系
    pB_R_h = np.hstack((pB_R, 1.0))
    pB_At_h = T_At_R @ pB_R_h
    pB_cam = pB_At_h[:3]
    
    # 检查在相机前方
    if pB_cam[2] <= 1e-6:
        return None  # 在后方或太接近
    
    # 投影到像素（使用OpenCV）
    objPts = pB_cam.reshape((1,1,3)).astype(np.float32)
    rvec = np.zeros(3, dtype=np.float32)
    tvec = np.zeros(3, dtype=np.float32)
    imgpts, _ = cv2.projectPoints(objPts, rvec, tvec, K, dist)
    pixel = imgpts[0,0].ravel()  # (u, v)
    
    return pixel, pB_cam


# ========== ROS2 Node ==========
class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')
        
        # ========== 参数声明 ==========
        self.declare_parameter('pB_topic', '/pB')
        self.declare_parameter('T_R_At_topic', '/T_R_At')
        self.declare_parameter('transform_msg_type', 'transform')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.05)
        self.declare_parameter('publish_topic', '/projected_pixel')
        
        # ========== 获取参数 ==========
        pB_topic = self.get_parameter('pB_topic').value
        T_R_At_topic = self.get_parameter('T_R_At_topic').value
        transform_msg_type = self.get_parameter('transform_msg_type').value
        queue_size = self.get_parameter('queue_size').value
        slop = self.get_parameter('slop').value
        publish_topic = self.get_parameter('publish_topic').value
        
        # ========== 订阅器（只需要2个）==========
        self.sub_pB = MFSubscriber(self, PointStamped, pB_topic)
        
        if transform_msg_type == 'transform':
            self.sub_T_R_At = MFSubscriber(self, TransformStamped, T_R_At_topic)
        elif transform_msg_type == 'array':
            self.sub_T_R_At = MFSubscriber(self, Float32MultiArray, T_R_At_topic)
        else:
            self.get_logger().error("Unsupported transform_msg_type")
            raise ValueError("Use 'transform' or 'array'")
        
        # ========== 时间同步器（只同步2个话题）==========
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_pB, self.sub_T_R_At],
            queue_size=queue_size,
            slop=slop)
        self.ts.registerCallback(self.synced_callback)
        
        # ========== 发布器 ==========
        self.pub_pixel = self.create_publisher(PointStamped, publish_topic, 10)
        
        # ========== 统计计数器 ==========
        self._success_count = 0
        self._fail_count = 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ProjectionNode 已初始化（简化测试版）")
        self.get_logger().info(f"订阅: pB={pB_topic}, T_R_At={T_R_At_topic}")
        self.get_logger().info(f"发布: {publish_topic}")
        self.get_logger().info(f"相机内参: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        self.get_logger().info(f"同步设置: queue={queue_size}, slop={slop}s")
        self.get_logger().info("等待同步消息...")
        self.get_logger().info("=" * 60)
    
    def extract_point(self, msg):
        """提取3D点"""
        return np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
    
    def extract_transform_matrix(self, msg):
        """
        提取4x4变换矩阵
        支持 TransformStamped 或 Float32MultiArray (16元素)
        """
        if isinstance(msg, TransformStamped):
            q = msg.transform.rotation
            R = quaternion_to_R(q.x, q.y, q.z, q.w)
            t = np.array([
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z
            ], dtype=float)
            return make_T(R, t)
        
        elif isinstance(msg, Float32MultiArray):
            data = np.array(msg.data, dtype=float)
            if data.size == 16:
                return data.reshape((4,4))
            else:
                raise ValueError("Float32MultiArray must have 16 elements")
        
        else:
            raise ValueError("Unsupported message type")
    
    def synced_callback(self, pB_msg, T_R_At_msg):
        """
        当2个消息时间同步后调用
        
        参数:
            pB_msg: 敌方在雷达系的位置
            T_R_At_msg: 我方相机在雷达系的位姿
        """
        try:
            # ========== 提取数据 ==========
            pB_R = self.extract_point(pB_msg)
            T_R_At = self.extract_transform_matrix(T_R_At_msg)
            
            # ========== 运行投影 ==========
            result = project_B_to_image(pB_R, T_R_At)
            
            if result is None:
                self._fail_count += 1
                if self._fail_count % 10 == 1:
                    self.get_logger().info(
                        f"目标不可见（在相机后方或太近）- 失败次数: {self._fail_count}")
                return
            
            pixel, pB_cam = result
            u, v = float(pixel[0]), float(pixel[1])
            depth = float(pB_cam[2])
            
            # ========== 发布投影结果 ==========
            out = PointStamped()
            out.header.stamp = pB_msg.header.stamp
            out.header.frame_id = "image"
            out.point.x = u
            out.point.y = v
            out.point.z = depth
            self.pub_pixel.publish(out)
            
            # ========== 统计和日志 ==========
            self._success_count += 1
            if self._success_count % 10 == 0:
                self.get_logger().info(
                    f"✓ [{self._success_count}] 投影成功 - "
                    f"像素: ({u:.1f}, {v:.1f}), 深度: {depth:.2f}m, "
                    f"目标在相机系: [{pB_cam[0]:.2f}, {pB_cam[1]:.2f}, {pB_cam[2]:.2f}]"
                )
        
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = ProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
