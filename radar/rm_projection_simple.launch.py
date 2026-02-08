"""
ROS2 Launch文件 - RM透视投影系统（简化测试版）

功能：
1. 启动 pointlio_to_projection_node（转换节点）
2. 启动 projection_node（投影节点）
3. 加载参数配置

使用方法：
ros2 launch <your_package> rm_projection_simple.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # ========== 声明启动参数 ==========
    
    # 配置文件路径
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='rm_projection_params_SIMPLE.yaml',
        description='参数配置文件路径'
    )
    
    # T_R_A0 文件路径
    t_r_a0_file_arg = DeclareLaunchArgument(
        'T_R_A0_file',
        default_value='T_B_to_A.txt',
        description='雷达站到相机初始系的变换矩阵文件'
    )
    
    # Point-LIO 里程计话题
    odometry_topic_arg = DeclareLaunchArgument(
        'odometry_topic',
        default_value='/Odometry',
        description='Point-LIO里程计话题名'
    )
    
    # pB 话题（敌方位置）
    pB_topic_arg = DeclareLaunchArgument(
        'pB_topic',
        default_value='/pB',
        description='敌方在雷达系的位置话题'
    )
    
    # 是否发布 pA
    publish_pA_arg = DeclareLaunchArgument(
        'publish_pA',
        default_value='true',
        description='是否发布我方在雷达系的位置（用于可视化）'
    )

    # 相机图像话题
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='相机图像话题'
    )

    # 叠加图像输出话题
    overlay_output_topic_arg = DeclareLaunchArgument(
        'overlay_output_topic',
        default_value='/projected_image',
        description='叠加后的图像话题'
    )

    # 是否启用图像叠加节点
    enable_overlay_arg = DeclareLaunchArgument(
        'enable_overlay',
        default_value='true',
        description='是否启动图像叠加节点'
    )

    # 是否启用海康相机采集节点
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='是否启动海康MVS相机采集节点'
    )

    # 海康MVS Python路径
    mvs_python_path_arg = DeclareLaunchArgument(
        'mvs_python_path',
        default_value='/opt/MVS/Samples/64/Python',
        description='海康MVS Python SDK路径'
    )

    # 相机设备索引
    camera_device_index_arg = DeclareLaunchArgument(
        'camera_device_index',
        default_value='0',
        description='海康相机设备索引'
    )

    # 图像frame_id
    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera',
        description='发布图像的frame_id'
    )
    
    # ========== 节点1: pointlio_to_projection_node ==========
    pointlio_to_projection_node = Node(
        package='radar',  # TODO: 替换为你的包名
        executable='pointlio_to_projection_node_SIMPLE.py',
        name='pointlio_to_projection_node',
        output='screen',
        parameters=[{
            'T_R_A0_file': LaunchConfiguration('T_R_A0_file'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'publish_pA': LaunchConfiguration('publish_pA'),
        }],
        # 可选：添加重映射
        # remappings=[
        #     ('/Odometry', '/your/custom/odom_topic'),
        # ]
    )
    
    # ========== 节点2: projection_node ==========
    projection_node = Node(
        package='radar',  # TODO: 替换为你的包名
        executable='core2_5ros_SIMPLE.py',
        name='projection_node',
        output='screen',
        parameters=[{
            'pB_topic': LaunchConfiguration('pB_topic'),
            'T_R_At_topic': '/T_R_At',
            'transform_msg_type': 'transform',
            'queue_size': 10,
            'slop': 0.05,
            'publish_topic': '/projected_pixel',
        }],
    )

    # ========== 节点3: projected_pixel_overlay_node ==========
    overlay_node = Node(
        package='radar',  # TODO: 替换为你的包名
        executable='projected_pixel_overlay_node.py',
        name='projected_pixel_overlay_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'pixel_topic': '/projected_pixel',
            'output_topic': LaunchConfiguration('overlay_output_topic'),
            'queue_size': 10,
            'slop': 0.05,
        }],
        condition=IfCondition(LaunchConfiguration('enable_overlay')),
    )

    # ========== 节点4: hikvision_mvs_camera_node ==========
    camera_node = Node(
        package='radar',  # TODO: 替换为你的包名
        executable='hikvision_mvs_camera_node.py',
        name='hikvision_mvs_camera_node',
        output='screen',
        parameters=[{
            'mvs_python_path': LaunchConfiguration('mvs_python_path'),
            'device_index': LaunchConfiguration('camera_device_index'),
            'image_topic': LaunchConfiguration('image_topic'),
            'frame_id': LaunchConfiguration('camera_frame_id'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )
    
    # ========== 返回启动描述 ==========
    return LaunchDescription([
        # 声明参数
        config_file_arg,
        t_r_a0_file_arg,
        odometry_topic_arg,
        pB_topic_arg,
        publish_pA_arg,
        image_topic_arg,
        overlay_output_topic_arg,
        enable_overlay_arg,
        enable_camera_arg,
        mvs_python_path_arg,
        camera_device_index_arg,
        camera_frame_id_arg,
        
        # 启动节点
        pointlio_to_projection_node,
        projection_node,
        overlay_node,
        camera_node,
    ])
