#!/usr/bin/env python3
"""
测试用敌方位置发布器

功能：
- 发布模拟的敌方位置到 /pB 话题
- 可以设置静态位置或动态移动

使用方法：
ros2 run <package> enemy_position_publisher.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import math


class EnemyPositionPublisher(Node):
    def __init__(self):
        super().__init__('enemy_position_publisher')
        
        # ========== 参数声明 ==========
        self.declare_parameter('pB_topic', '/pB')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('mode', 'static')  # 'static' or 'dynamic'
        
        # 静态位置参数
        self.declare_parameter('x', 2.0)
        self.declare_parameter('y', 1.0)
        self.declare_parameter('z', 0.5)
        
        # 动态运动参数
        self.declare_parameter('circle_radius', 2.0)
        self.declare_parameter('circle_speed', 0.5)  # rad/s
        
        # ========== 获取参数 ==========
        pB_topic = self.get_parameter('pB_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.mode = self.get_parameter('mode').value
        
        self.static_x = self.get_parameter('x').value
        self.static_y = self.get_parameter('y').value
        self.static_z = self.get_parameter('z').value
        
        self.circle_radius = self.get_parameter('circle_radius').value
        self.circle_speed = self.get_parameter('circle_speed').value
        
        # ========== 发布器 ==========
        self.publisher = self.create_publisher(PointStamped, pB_topic, 10)
        
        # ========== 定时器 ==========
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_callback)
        
        # ========== 状态变量 ==========
        self.angle = 0.0
        self.count = 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("敌方位置发布器已启动")
        self.get_logger().info(f"话题: {pB_topic}")
        self.get_logger().info(f"频率: {publish_rate} Hz")
        self.get_logger().info(f"模式: {self.mode}")
        if self.mode == 'static':
            self.get_logger().info(f"静态位置: ({self.static_x}, {self.static_y}, {self.static_z})")
        else:
            self.get_logger().info(f"圆周运动: 半径={self.circle_radius}m, 速度={self.circle_speed}rad/s")
        self.get_logger().info("=" * 60)
    
    def publish_callback(self):
        """发布敌方位置"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'radar'
        
        if self.mode == 'static':
            # 静态位置
            msg.point.x = self.static_x
            msg.point.y = self.static_y
            msg.point.z = self.static_z
        
        elif self.mode == 'circle':
            # 圆周运动（XY平面）
            msg.point.x = self.circle_radius * math.cos(self.angle)
            msg.point.y = self.circle_radius * math.sin(self.angle)
            msg.point.z = self.static_z
            
            self.angle += self.circle_speed * 0.1  # 假设10Hz
            if self.angle > 2 * math.pi:
                self.angle -= 2 * math.pi
        
        elif self.mode == 'sine':
            # 正弦波运动
            msg.point.x = 2.0
            msg.point.y = math.sin(self.angle) * 1.0
            msg.point.z = 0.5
            
            self.angle += 0.1
        
        else:
            # 默认静态
            msg.point.x = self.static_x
            msg.point.y = self.static_y
            msg.point.z = self.static_z
        
        self.publisher.publish(msg)
        
        # 日志输出（降低频率）
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(
                f"[{self.count}] 发布敌方位置: "
                f"({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = EnemyPositionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
