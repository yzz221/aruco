#!/usr/bin/env python3
"""
pointlio_to_projection_node - 简化测试版本

场景：
- 没有机器人，没有 base_link
- 只有 Lidar(Mid-360) 和 Camera 刚性固定（假设重合）
- 相机第一帧（标定ArUco时）作为 odom 原点 A0

功能：
1. 接收 Point-LIO 的里程计（Lidar在odom系的位姿）
2. 第一帧设为相机初始系 A0
3. 计算相机在雷达系 R 的位姿
4. 发布 T_R_At 给投影节点
"""
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, TransformStamped


class PointLioToProjectionNode(Node):
    def __init__(self):
        super().__init__('pointlio_to_projection_node')
        
        # ========== 参数声明 ==========
        self.declare_parameter('T_R_A0_file', 'T_B_to_A.txt')
        self.declare_parameter('odometry_topic', '/Odometry')
        self.declare_parameter('publish_pA', True)
        self.declare_parameter('apply_odom_to_camera_axis', True)
        self.declare_parameter('odom_axes', 'x_forward_y_left_z_up')
        self.declare_parameter('camera_axes', 'x_right_y_down_z_forward')
        self.declare_parameter('use_yaw_only', True)
        self.declare_parameter('apply_camera_extrinsic', False)
        self.declare_parameter('camera_extrinsic_rpy_deg', [0.0, 0.0, 0.0])
        
        # ========== 获取参数 ==========
        T_R_A0_file = self.get_parameter('T_R_A0_file').value
        odometry_topic = self.get_parameter('odometry_topic').value
        self.publish_pA_enabled = self.get_parameter('publish_pA').value
        self.apply_axis_convert = bool(self.get_parameter('apply_odom_to_camera_axis').value)
        self.odom_axes = str(self.get_parameter('odom_axes').value)
        self.camera_axes = str(self.get_parameter('camera_axes').value)
        self.use_yaw_only = bool(self.get_parameter('use_yaw_only').value)
        self.apply_camera_extrinsic = bool(self.get_parameter('apply_camera_extrinsic').value)
        self.camera_extrinsic_rpy_deg = list(self.get_parameter('camera_extrinsic_rpy_deg').value)
        
        # ========== 加载 T_R_A0 ==========
        # T_R_A0 = T_B_to_A (从 B_board.py 输出)
        # 雷达站 R → 相机初始系 A0
        self.T_R_A0 = self.load_transform_matrix(T_R_A0_file)
        if self.T_R_A0 is None:
            self.get_logger().error(f"Failed to load T_R_A0 from {T_R_A0_file}")
            raise RuntimeError("Transform matrix loading failed")
        
        self.get_logger().info(f"✓ 已加载 T_R_A0 (雷达站→相机初始系A0):\n{self.T_R_A0}")
        
        # ========== 初始化标志 ==========
        self.initial_pose_set = False
        self.T_odom_A0 = np.eye(4)  # Point-LIO的odom系 → 相机初始系A0
        self.T_A0_odom = np.eye(4)  # 逆变换
        
        # ========== 订阅 Point-LIO 里程计 ==========
        self.odom_sub = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odometry_callback,
            10
        )
        
        # ========== 发布器 ==========
        if self.publish_pA_enabled:
            self.pub_pA = self.create_publisher(PointStamped, '/pA', 10)
        
        self.pub_T_R_At = self.create_publisher(TransformStamped, '/T_R_At', 10)
        
        # ========== 日志计数器 ==========
        self._log_counter = 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("PointLioToProjectionNode 已初始化（简化测试版）")
        self.get_logger().info(f"订阅: {odometry_topic}")
        self.get_logger().info(f"发布: /T_R_At" + (" 和 /pA" if self.publish_pA_enabled else ""))
        self.get_logger().info("假设: Lidar 和 Camera 重合")
        self.get_logger().info(
            f"坐标轴转换: {self.apply_axis_convert} | odom轴: {self.odom_axes} -> camera轴: {self.camera_axes}"
        )
        self.get_logger().info(f"调试开关: use_yaw_only={self.use_yaw_only}")
        self.get_logger().info(
            f"固定外参: enabled={self.apply_camera_extrinsic}, rpy_deg={self.camera_extrinsic_rpy_deg}"
        )
        self.get_logger().info("等待第一帧里程计以设定相机初始系 A0...")
        self.get_logger().info("=" * 60)

        # ========== 轴系转换矩阵（里程计/雷达到相机） ==========
        self.T_lidar_cam = self._build_lidar_to_camera_transform()
        self._log_axis_mapping_once()
    
    def load_transform_matrix(self, filename):
        """从文件加载4x4变换矩阵"""
        try:
            data = np.loadtxt(filename)
            if data.shape == (4, 4):
                return data
            elif data.shape == (16,):
                return data.reshape(4, 4)
            else:
                self.get_logger().error(f"Invalid matrix shape: {data.shape}")
                return None
        except Exception as e:
            self.get_logger().error(f"Error loading matrix: {e}")
            return None
    
    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """四元数转旋转矩阵"""
        rot = R.from_quat([qx, qy, qz, qw])
        return rot.as_matrix()

    def _keep_yaw_only(self, R_mat):
        """仅保留yaw(绕Z轴)的旋转，忽略roll/pitch"""
        # ZYX顺序：从旋转矩阵提取yaw
        yaw = float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        R_yaw = np.array(
            [
                [cy, -sy, 0.0],
                [sy,  cy, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return R_yaw
    
    def rotation_matrix_to_quaternion(self, R_mat):
        """旋转矩阵转四元数"""
        rot = R.from_matrix(R_mat)
        return rot.as_quat()  # [x, y, z, w]

    def _build_lidar_to_camera_transform(self):
        """
        构建 Lidar/里程计坐标系 -> 相机坐标系 的固定旋转

        默认假设：
        - 里程计轴：x前、y左、z上
        - 相机轴：x右、y下、z前

        则：
        x_cam = -y_odom
        y_cam = -z_odom
        z_cam =  x_odom
        """
        if not self.apply_axis_convert:
            T = np.eye(4)
            return T

        if self.odom_axes != 'x_forward_y_left_z_up' or self.camera_axes != 'x_right_y_down_z_forward':
            self.get_logger().warn(
                "当前仅内置 x_forward_y_left_z_up -> x_right_y_down_z_forward 的轴系转换，将按内置矩阵处理。"
            )

        R_cam_odom = np.array(
            [
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
                [1.0, 0.0, 0.0],
            ],
            dtype=float,
        )

        # 我们需要 lidar <- cam 的变换（odom<-lidar<-cam），所以用转置
        R_lidar_cam = R_cam_odom.T

        R_lidar_cam = self._apply_camera_extrinsic(R_lidar_cam)

        T = np.eye(4)
        T[:3, :3] = R_lidar_cam
        return T

    def _apply_camera_extrinsic(self, R_lidar_cam):
        """应用固定外参旋转（roll/pitch/yaw，单位度）"""
        if not self.apply_camera_extrinsic:
            return R_lidar_cam

        if len(self.camera_extrinsic_rpy_deg) != 3:
            self.get_logger().warn("camera_extrinsic_rpy_deg 长度不为3，忽略外参旋转")
            return R_lidar_cam

        roll_deg, pitch_deg, yaw_deg = self.camera_extrinsic_rpy_deg
        R_extra = R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()

        # 约定：在完成轴映射后，再施加相机固定外参
        return R_lidar_cam @ R_extra

    def _log_axis_mapping_once(self):
        """打印一次轴映射的数值验证，便于确认方向是否正确"""
        if not self.apply_axis_convert:
            self.get_logger().info("轴系转换未启用：使用里程计坐标系作为相机系")
            return

        R_lidar_cam = self.T_lidar_cam[:3, :3]
        ex = np.array([1.0, 0.0, 0.0])
        ey = np.array([0.0, 1.0, 0.0])
        ez = np.array([0.0, 0.0, 1.0])

        cam_ex = R_lidar_cam.T @ ex
        cam_ey = R_lidar_cam.T @ ey
        cam_ez = R_lidar_cam.T @ ez

        self.get_logger().info("轴映射检查（odom单位轴 -> cam单位轴）:")
        self.get_logger().info(f"  odom +X -> cam [{cam_ex[0]:+.1f}, {cam_ex[1]:+.1f}, {cam_ex[2]:+.1f}]")
        self.get_logger().info(f"  odom +Y -> cam [{cam_ey[0]:+.1f}, {cam_ey[1]:+.1f}, {cam_ey[2]:+.1f}]")
        self.get_logger().info(f"  odom +Z -> cam [{cam_ez[0]:+.1f}, {cam_ez[1]:+.1f}, {cam_ez[2]:+.1f}]")
    
    def odometry_callback(self, msg: Odometry):
        """
        处理 Point-LIO 里程计消息
        
        Point-LIO 输出的 Odometry:
        - msg.pose.pose 是 base_link 在 odom 系的位姿
        - 但我们的场景没有 base_link，直接当作 Lidar/Camera 的位姿
        
        流程：
        1. 提取当前 Lidar/Camera 在 Point-LIO odom 系的位姿 T_odom_lidar
        2. 第一帧：设 T_odom_A0 = T_odom_lidar（定义 A0）
        3. 后续帧：计算 T_A0_lidar = inv(T_odom_A0) @ T_odom_lidar
        4. 计算雷达系位姿：T_R_lidar = T_R_A0 @ T_A0_lidar
        5. 发布 T_R_At (因为 Lidar=Camera，所以就是相机位姿)
        """
        try:
            # ========== 步骤1: 提取 Lidar/Camera 在 Point-LIO odom 系的位姿 ==========
            position_odom = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            
            orientation = msg.pose.pose.orientation
            R_odom_lidar = self.quaternion_to_rotation_matrix(
                orientation.x, orientation.y, orientation.z, orientation.w
            )

            if self.use_yaw_only:
                R_odom_lidar = self._keep_yaw_only(R_odom_lidar)
            
            # 构建 T_odom_lidar (Point-LIO的odom系 → Lidar/Camera系)
            T_odom_lidar = np.eye(4)
            T_odom_lidar[:3, :3] = R_odom_lidar
            T_odom_lidar[:3, 3] = position_odom

            # ========== 轴系转换：把里程计/Lidar坐标系对齐到相机坐标系 ==========
            # T_odom_cam = T_odom_lidar @ T_lidar_cam (odom -> cam)
            T_odom_cam = T_odom_lidar @ self.T_lidar_cam
            
            # ========== 步骤2: 第一帧设为相机初始系 A0 ==========
            if not self.initial_pose_set:
                # 第一帧到达，定义此时的 Lidar/Camera 位姿为 A0
                self.T_odom_A0 = T_odom_cam.copy()
                self.T_A0_odom = np.linalg.inv(self.T_odom_A0)
                self.initial_pose_set = True
                
                self.get_logger().info("=" * 60)
                self.get_logger().info("✓ 相机初始系 A0 已设定（对齐到第一帧）")
                self.get_logger().info(f"  A0 在 Point-LIO odom 系的位置: {position_odom}")
                self.get_logger().info(f"  A0 在 Point-LIO odom 系的姿态(quat): "
                    f"[{orientation.x:.4f}, {orientation.y:.4f}, "
                    f"{orientation.z:.4f}, {orientation.w:.4f}]")
                self.get_logger().info("  从现在开始，所有位姿都相对于此 A0 计算")
                self.get_logger().info("=" * 60)
            
            # ========== 步骤3: 计算相机相对于 A0 的位姿 ==========
            # T_A0_lidar: 从相机初始系 A0 到当前 Lidar/Camera
            T_A0_lidar = self.T_A0_odom @ T_odom_cam
            
            # ========== 步骤4: 计算相机在雷达系 R 的位姿 ==========
            # T_R_lidar: 从雷达站 R 到当前 Lidar/Camera
            # 因为 Lidar = Camera (假设重合)，所以这就是 T_R_At
            T_R_camera = self.T_R_A0 @ T_A0_lidar
            
            # ========== 步骤5: 发布数据 ==========
            
            # 5.1 可选：发布相机在雷达系的位置（用于可视化/调试）
            if self.publish_pA_enabled:
                position_R = T_R_camera[:3, 3]
                pA_msg = PointStamped()
                pA_msg.header.stamp = msg.header.stamp
                pA_msg.header.frame_id = "radar"
                pA_msg.point.x = float(position_R[0])
                pA_msg.point.y = float(position_R[1])
                pA_msg.point.z = float(position_R[2])
                self.pub_pA.publish(pA_msg)
            
            # 5.2 关键：发布 T_R_At (相机在雷达系的完整位姿)
            quat = self.rotation_matrix_to_quaternion(T_R_camera[:3, :3])
            trans = T_R_camera[:3, 3]
            
            T_R_At_msg = TransformStamped()
            T_R_At_msg.header.stamp = msg.header.stamp
            T_R_At_msg.header.frame_id = "radar"
            T_R_At_msg.child_frame_id = "camera_current"
            
            # 平移
            T_R_At_msg.transform.translation.x = float(trans[0])
            T_R_At_msg.transform.translation.y = float(trans[1])
            T_R_At_msg.transform.translation.z = float(trans[2])
            
            # 旋转
            T_R_At_msg.transform.rotation.x = float(quat[0])
            T_R_At_msg.transform.rotation.y = float(quat[1])
            T_R_At_msg.transform.rotation.z = float(quat[2])
            T_R_At_msg.transform.rotation.w = float(quat[3])
            
            self.pub_T_R_At.publish(T_R_At_msg)
            
            # ========== 步骤6: 日志输出（降低频率） ==========
            self._log_counter += 1
            if self._log_counter % 50 == 0:
                # 额外输出：映射后的平移分量，便于检查水平移动方向
                delta_A0 = T_A0_lidar[:3, 3]
                self.get_logger().info(
                    f"[{self._log_counter}] 已发布 T_R_At - "
                    f"位置: [{trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f}], "
                    f"姿态: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}] | "
                    f"A0->当前平移(cam轴): [{delta_A0[0]:+.3f}, {delta_A0[1]:+.3f}, {delta_A0[2]:+.3f}]"
                )
        
        except Exception as e:
            self.get_logger().error(f"Error in odometry_callback: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PointLioToProjectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
