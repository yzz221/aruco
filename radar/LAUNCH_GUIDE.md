# RM透视投影系统 - 完整启动指南

## 📁 文件清单

```
工作目录/
├── T_B_to_A.txt                          # B_board.py标定输出（必需）
├── rm_projection_params_SIMPLE.yaml      # 参数配置文件
├── pointlio_to_projection_node_SIMPLE.py # 转换节点
├── core2_5ros_SIMPLE.py                  # 投影节点
├── projected_pixel_overlay_node.py       # 图像叠加节点
├── enemy_position_publisher.py           # 测试用敌方位置发布器
└── rm_projection_simple.launch.py        # 启动文件
```

---

## 🚀 方法1: 使用 Launch 文件启动（推荐）

### 步骤1: 确保文件已安装到 ROS2 包

```bash
# 1. 将文件复制到你的 ROS2 包中
cd ~/ros2_ws/src/your_package/

# 2. 创建必要的目录
mkdir -p scripts config launch

# 3. 复制文件
cp pointlio_to_projection_node_SIMPLE.py scripts/
cp core2_5ros_SIMPLE.py scripts/
cp projected_pixel_overlay_node.py scripts/
cp enemy_position_publisher.py scripts/
cp rm_projection_simple.launch.py launch/
cp rm_projection_params_SIMPLE.yaml config/

# 4. 添加可执行权限
chmod +x scripts/*.py

# 5. 编辑 setup.py，添加：
# data_files=[
#     ...
#     (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
#     (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
# ],
# 
# entry_points={
#     'console_scripts': [
#         'pointlio_to_projection_node_SIMPLE.py = your_package.pointlio_to_projection_node_SIMPLE:main',
#         'core2_5ros_SIMPLE.py = your_package.core2_5ros_SIMPLE:main',
#         'projected_pixel_overlay_node.py = your_package.projected_pixel_overlay_node:main',
#         'enemy_position_publisher.py = your_package.enemy_position_publisher:main',
#     ],
# },

# 6. 编译
cd ~/ros2_ws
colcon build --packages-select your_package
source install/setup.bash
```

### 步骤2: 准备标定文件

```bash
# 将 T_B_to_A.txt 复制到工作目录
cp /path/to/T_B_to_A.txt ~/ros2_ws/
cd ~/ros2_ws/
```

### 步骤3: 启动 Point-LIO（终端1）

```bash
# 启动 Point-LIO
ros2 launch point_lio mapping.launch.py

# 确保发布了 /Odometry 话题
ros2 topic list | grep Odometry
```

### 步骤4: 启动投影系统（终端2）

```bash
cd ~/ros2_ws/

# ⚠️ 重要：在启动之前，将相机/传感器放在标定 ArUco 的位置！
# 因为第一帧会被定义为 A0

ros2 launch your_package rm_projection_simple.launch.py \
    T_R_A0_file:=T_B_to_A.txt

# 如果相机话题不是 /camera/image_raw，请追加：
# image_topic:=/你的相机话题

# 如需启动海康MVS相机采集节点，可追加：
# enable_camera:=true mvs_python_path:=/opt/MVS/Samples/64/Python camera_device_index:=0
```

### 步骤5: 发布敌方位置（终端3）

#### 方式A - 使用测试节点（动态运动）
```bash
# 圆周运动
ros2 run your_package enemy_position_publisher.py \
    --ros-args \
    -p mode:=circle \
    -p circle_radius:=2.0 \
    -p circle_speed:=0.5

# 或静态位置
ros2 run your_package enemy_position_publisher.py \
    --ros-args \
    -p mode:=static \
    -p x:=2.0 \
    -p y:=1.0 \
    -p z:=0.5
```

#### 方式B - 使用命令行（简单测试）
```bash
ros2 topic pub /pB geometry_msgs/PointStamped \
"{header: {frame_id: 'radar'}, 
  point: {x: 2.0, y: 1.0, z: 0.5}}" \
-r 10
```

### 步骤6: 查看结果（终端4）

```bash
# 查看投影结果
ros2 topic echo /projected_pixel

# 输出示例：
# header:
#   stamp:
#     sec: 1234567890
#     nanosec: 123456789
#   frame_id: image
# point:
#   x: 1234.5  # 像素列 (u)
#   y: 678.9   # 像素行 (v)
#   z: 3.45    # 深度 (米)

### 叠加图像（可选）
启动后会发布 `/projected_image`，可在你的图像查看工具中打开。
如需关闭叠加节点，可在启动时添加 `enable_overlay:=false`。
```

---

## 🔧 方法2: 手动逐个启动（调试用）

### 终端1: Point-LIO
```bash
ros2 launch point_lio mapping.launch.py
```

### 终端2: 转换节点
```bash
cd ~/ros2_ws/

ros2 run your_package pointlio_to_projection_node_SIMPLE.py \
    --ros-args \
    -p T_R_A0_file:=T_B_to_A.txt \
    -p odometry_topic:=/Odometry \
    -p publish_pA:=true
```

### 终端3: 投影节点
```bash
ros2 run your_package core2_5ros_SIMPLE.py \
    --ros-args \
    -p pB_topic:=/pB \
    -p T_R_At_topic:=/T_R_At \
    -p transform_msg_type:=transform \
    -p queue_size:=10 \
    -p slop:=0.05 \
    -p publish_topic:=/projected_pixel
```

### 终端4: 敌方位置发布
```bash
ros2 run your_package enemy_position_publisher.py \
    --ros-args \
    -p mode:=static \
    -p x:=2.0 \
    -p y:=1.0 \
    -p z:=0.5
```

### 终端5: 查看结果
```bash
ros2 topic echo /projected_pixel
```

---

## 📊 监控和调试

### 检查话题列表
```bash
ros2 topic list

# 应该看到：
# /Odometry          (Point-LIO输出)
# /pA                (我方位置，可选)
# /T_R_At            (相机在雷达系位姿)
# /pB                (敌方位置)
# /projected_pixel   (投影结果)
```

### 检查话题频率
```bash
ros2 topic hz /Odometry
ros2 topic hz /T_R_At
ros2 topic hz /pB
ros2 topic hz /projected_pixel
```

### 检查话题内容
```bash
# 查看里程计
ros2 topic echo /Odometry --once

# 查看相机位姿
ros2 topic echo /T_R_At --once

# 查看敌方位置
ros2 topic echo /pB --once

# 查看投影结果
ros2 topic echo /projected_pixel
```

### 查看TF树
```bash
ros2 run tf2_tools view_frames
evince frames.pdf

# 或实时查看
ros2 run tf2_ros tf2_echo odom base_link
```

### 查看节点信息
```bash
ros2 node list
ros2 node info /pointlio_to_projection_node
ros2 node info /projection_node
```

---

## 🎯 测试场景

### 场景1: 静态敌方，移动相机
```bash
# 1. 启动系统
ros2 launch your_package rm_projection_simple.launch.py

# 2. 发布静态敌方位置
ros2 topic pub /pB geometry_msgs/PointStamped \
"{header: {frame_id: 'radar'}, point: {x: 2.0, y: 0.0, z: 0.5}}" -r 10

# 3. 移动传感器（相机+Lidar）
#    观察 /projected_pixel 的变化
```

### 场景2: 动态敌方，静态相机
```bash
# 1. 启动系统
ros2 launch your_package rm_projection_simple.launch.py

# 2. 保持相机静止

# 3. 发布圆周运动的敌方
ros2 run your_package enemy_position_publisher.py \
    --ros-args -p mode:=circle -p circle_radius:=2.0
```

### 场景3: 双方都运动
```bash
# 1. 启动系统
ros2 launch your_package rm_projection_simple.launch.py

# 2. 发布动态敌方
ros2 run your_package enemy_position_publisher.py \
    --ros-args -p mode:=circle

# 3. 移动相机
#    观察投影结果
```

---

## ⚠️ 常见问题排查

### 问题1: 没有输出到 /projected_pixel

**检查**:
```bash
# 1. 确认所有话题都在发布
ros2 topic hz /Odometry
ros2 topic hz /T_R_At
ros2 topic hz /pB

# 2. 查看投影节点日志
ros2 node info /projection_node

# 3. 检查时间同步
ros2 topic echo /pB --field header.stamp
ros2 topic echo /T_R_At --field header.stamp
```

**解决**:
```bash
# 增大时间同步容差
ros2 param set /projection_node slop 0.2
```

### 问题2: "B not visible (behind camera)"

**原因**: 敌方在相机后方

**检查**:
```bash
# 查看相机位姿
ros2 topic echo /T_R_At

# 查看敌方位置
ros2 topic echo /pB

# 计算相对位置
```

**解决**:
- 调整敌方位置（改 pB 的坐标）
- 或移动相机朝向敌方

### 问题3: 投影坐标异常（超出图像范围）

**检查**:
```bash
# 查看投影结果
ros2 topic echo /projected_pixel

# 检查相机内参是否正确
# 编辑 core2_5ros_SIMPLE.py 中的 fx, fy, cx, cy
```

### 问题4: 第一帧位置不对

**解决**:
```bash
# 1. 停止节点
# 2. 将相机移动到标定 ArUco 的位置
# 3. 重新启动转换节点
ros2 run your_package pointlio_to_projection_node_SIMPLE.py ...
```

---

## 📝 参数调优

### 调整时间同步容差
```bash
# 如果消息不同步，增大 slop
ros2 param set /projection_node slop 0.1  # 100ms

# 或启动时设置
ros2 run ... core2_5ros_SIMPLE.py --ros-args -p slop:=0.1
```

### 调整队列大小
```bash
# 增大队列可以缓存更多消息
ros2 param set /projection_node queue_size 20
```

### 开启调试日志
```bash
ros2 run ... --ros-args --log-level DEBUG
```

---

## 🎉 成功标志

系统正常运行时，你应该看到：

### 转换节点日志
```
✓ 已加载 T_R_A0 (雷达站→相机初始系A0)
✓ 相机初始系 A0 已设定（对齐到第一帧）
  A0 在 Point-LIO odom 系的位置: [0.123, 0.456, 0.789]
[50] 已发布 T_R_At - 位置: [1.234, 0.567, 0.890]...
```

### 投影节点日志
```
ProjectionNode 已初始化（简化测试版）
订阅: pB=/pB, T_R_At=/T_R_At
✓ [10] 投影成功 - 像素: (1234.5, 678.9), 深度: 3.45m
```

### 话题数据正常
```bash
$ ros2 topic hz /projected_pixel
average rate: 10.023
    min: 0.098s max: 0.102s std dev: 0.00123s window: 10
```

---

## 🔄 快速重启流程

```bash
# 1. 停止所有节点（Ctrl+C）

# 2. 确保相机在标定位置

# 3. 一键启动
ros2 launch your_package rm_projection_simple.launch.py

# 4. 发布敌方位置
ros2 run your_package enemy_position_publisher.py

# 5. 查看结果
ros2 topic echo /projected_pixel
```

祝测试顺利！🎯
