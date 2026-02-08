# 简化测试版本 - 使用说明

## 📋 系统配置

### 硬件组成
```
传感器模块（刚性固定）
├─ Livox Mid-360 (激光雷达)
└─ RGB 相机
```

**假设**: Lidar 和 Camera 坐标系重合（简化测试）

### 坐标系定义
```
R (Radar)     - 雷达站固定参考系
A0 (Camera0)  - 相机第一帧（标定ArUco时）= odom原点
At (Current)  - 当前相机/Lidar位姿
```

---

## 🚀 快速启动指南

### 步骤1: 准备标定文件

运行你的 `B_board.py` 生成雷达站到相机的变换：

```bash
python3 B_board.py
# 输出: T_B_to_A.txt
```

将 `T_B_to_A.txt` 放在工作目录下。

---

### 步骤2: 启动 Point-LIO

```bash
ros2 launch point_lio mapping.launch.py
```

确保发布了 `/Odometry` 话题。

---

### 步骤3: 启动转换节点

```bash
cd /path/to/your/workspace
chmod +x pointlio_to_projection_node_SIMPLE.py

ros2 run <your_package> pointlio_to_projection_node_SIMPLE.py \
    --ros-args \
    -p T_R_A0_file:=T_B_to_A.txt \
    -p odometry_topic:=/Odometry \
    -p publish_pA:=true \
    -p apply_odom_to_camera_axis:=true \
    -p use_yaw_only:=true \
    -p apply_camera_extrinsic:=false \
    -p camera_extrinsic_rpy_deg:="[0.0, 0.0, 0.0]"

如果你的里程计坐标轴与相机坐标轴不同（常见：里程计 x前y左z上，
相机 x右y下z前），请保持 `apply_odom_to_camera_axis:=true`。
若两者轴一致，可设为 `false`。

如果旋转误差很大，可暂时设置 `use_yaw_only:=true`，
只保留 yaw（绕Z轴），忽略 roll/pitch，
用于判断误差是否来自IMU或外参。

如果确定相机相对雷达/里程计存在固定夹角，可设置固定外参旋转：
`apply_camera_extrinsic:=true` 并给出 `camera_extrinsic_rpy_deg`（XYZ顺序，单位度）。
```

**第一次运行时的重要提示**：
- 程序启动后会等待第一帧 `/Odometry`
- **第一帧到达的位置就是相机初始系 A0**
- 确保第一帧时，相机处于你标定 ArUco 的位置！

---

### 步骤4: 提供敌方位置

需要发布敌方在雷达系的位置（话题 `/pB`）。

**测试方法A - 使用命令行发布**：
```bash
ros2 topic pub /pB geometry_msgs/PointStamped \
"{header: {frame_id: 'radar'}, 
  point: {x: 1.0, y: 0.5, z: 0.0}}"
```

**测试方法B - 编写简单的发布节点**：
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class EnemyPublisher(Node):
    def __init__(self):
        super().__init__('enemy_publisher')
        self.pub = self.create_publisher(PointStamped, '/pB', 10)
        self.timer = self.create_timer(0.1, self.publish_enemy)
        
    def publish_enemy(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'radar'
        # 假设敌方在雷达系下的位置（修改这里测试）
        msg.point.x = 2.0
        msg.point.y = 1.0
        msg.point.z = 0.5
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = EnemyPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

### 步骤5: 启动投影节点

```bash
chmod +x core2_5ros_SIMPLE.py

ros2 run <your_package> core2_5ros_SIMPLE.py \
    --ros-args \
    -p pB_topic:=/pB \
    -p T_R_At_topic:=/T_R_At \
    -p publish_topic:=/projected_pixel
```

---

### 步骤6: 查看结果

查看投影结果：
```bash
ros2 topic echo /projected_pixel
```

输出示例：
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: image
point:
  x: 1234.5  # 像素列 (u)
  y: 678.9   # 像素行 (v)
  z: 3.45    # 深度 (米)

---

## 🖼️ 图像叠加显示（可选）

如果你没有单独的相机驱动节点，可使用海康MVS采集节点发布图像：

```bash
chmod +x hikvision_mvs_camera_node.py

ros2 run <your_package> hikvision_mvs_camera_node.py \
    --ros-args \
    -p mvs_python_path:=/opt/MVS/Samples/64/Python \
    -p device_index:=0 \
    -p image_topic:=/camera/image_raw
```

然后再启动叠加节点：

如果需要把像素点投影到相机画面上，可启动叠加节点：

```bash
chmod +x projected_pixel_overlay_node.py

ros2 run <your_package> projected_pixel_overlay_node.py \
        --ros-args \
        -p image_topic:=/camera/image_raw \
        -p pixel_topic:=/projected_pixel \
        -p output_topic:=/projected_image \
        -p show_window:=false
```

查看叠加后的图像：
```bash
ros2 topic echo /projected_image
```

如果你使用的是其它海康USB相机话题（如 `/hikvision/image_raw`），
请把 `image_topic` 改成实际话题名。
```

---

## 🔍 调试技巧

### 1. 检查话题是否发布

```bash
# 检查 Point-LIO 输出
ros2 topic echo /Odometry

# 检查转换节点输出
ros2 topic echo /T_R_At

# 检查敌方位置
ros2 topic echo /pB

# 检查投影结果
ros2 topic echo /projected_pixel
```

### 2. 检查话题频率

```bash
ros2 topic hz /Odometry
ros2 topic hz /T_R_At
ros2 topic hz /pB
```

### 3. 查看TF树

```bash
ros2 run tf2_tools view_frames
# 或
ros2 run tf2_ros tf2_echo odom base_link
```

### 4. 日志级别

如果需要更详细的调试信息：
```bash
ros2 run <pkg> pointlio_to_projection_node_SIMPLE.py \
    --ros-args --log-level DEBUG
```

---

## ⚠️ 常见问题

### Q1: "B not visible (behind camera)"
**原因**: 敌方在相机后方
**解决**: 
- 检查 pB 的坐标是否正确
- 检查 T_R_At 是否正确
- 移动相机或调整敌方位置

### Q2: 第一帧设置不对
**原因**: 第一帧时相机不在标定位置
**解决**: 
1. 删除节点
2. 把相机移动到标定 ArUco 的位置
3. 重新启动节点
4. 等待第一帧设定 A0

### Q3: 投影坐标不准
**可能原因**:
- 相机内参不对（检查 core2_5ros_SIMPLE.py 中的 K 和 dist）
- T_B_to_A 标定不准
- Lidar 和 Camera 实际上不重合（需要标定外参）

### Q4: 消息不同步
**检查**:
```bash
# 查看消息时间戳
ros2 topic echo /pB --field header.stamp
ros2 topic echo /T_R_At --field header.stamp
```
**调整同步参数**:
```bash
-p slop:=0.1  # 增加时间容差到 100ms
```

---

## 📊 数据流示意图

```
Point-LIO
    ↓ /Odometry
    ↓ (Lidar/Camera 在 odom 系的位姿)
    ↓
pointlio_to_projection_node
    ├─ 第一帧：定义 A0 = odom 原点
    ├─ 计算：T_A0_camera = inv(T_odom_A0) @ T_odom_camera
    ├─ 计算：T_R_camera = T_R_A0 @ T_A0_camera
    └─ 发布：/T_R_At (相机在雷达系的位姿)
    
雷达站 (或其他源)
    ↓ /pB
    ↓ (敌方在雷达系的位置)
    ↓
core2_5ros
    ├─ 接收：/pB, /T_R_At
    ├─ 计算：T_At_R = inv(T_R_At)
    ├─ 变换：pB_cam = T_At_R @ pB_R
    ├─ 投影：(u, v) = project(pB_cam)
    └─ 发布：/projected_pixel
```

---

## 🔧 下一步优化

### 1. 标定 Lidar-Camera 外参
如果 Lidar 和 Camera 实际上不重合，需要标定 T_lidar_camera：
- 使用 lidar-camera 联合标定工具
- 或手动测量相对位置

### 2. 添加可视化
- 在图像上绘制投影点
- 发布 TF 树
- 使用 RViz 可视化

### 3. 优化性能
- 调整消息队列大小
- 优化同步策略
- 添加数据缓存

---

## 📝 文件清单

```
工作目录/
├── T_B_to_A.txt                          # B_board.py 输出
├── pointlio_to_projection_node_SIMPLE.py # 转换节点
├── core2_5ros_SIMPLE.py                  # 投影节点
└── (可选) enemy_publisher.py             # 测试用的敌方位置发布器
```

---

## 🎯 成功标志

系统正常运行时，你应该能看到：

1. **转换节点日志**:
```
✓ 相机初始系 A0 已设定（对齐到第一帧）
[50] 已发布 T_R_At - 位置: [1.234, 0.567, 0.890]...
```

2. **投影节点日志**:
```
✓ [10] 投影成功 - 像素: (1234.5, 678.9), 深度: 3.45m
```

3. **话题数据正常**:
```bash
ros2 topic hz /T_R_At      # 应该有稳定频率
ros2 topic hz /projected_pixel  # 应该有输出
```

祝测试顺利！🚀
