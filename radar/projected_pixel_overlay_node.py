#!/usr/bin/env python3
"""
projected_pixel_overlay_node - 合并海康相机采集与投影像素叠加的单一节点

功能：
1. 使用海康 MVS SDK 打开相机并采集 BGR 图像（兼容原有 `hikvision_mvs_camera_node.py` 参数）
2. 发布原始图像到 `image_topic`（默认 `/camera/image_raw`），以保持向后兼容
3. 订阅投影像素（`pixel_topic`，geometry_msgs/PointStamped），在每帧上叠加最近的像素并发布到 `output_topic`
4. 支持在窗口中显示叠加结果（可选）

说明：此实现把相机采集与叠加逻辑放在同一个 ROS 节点中，省去跨节点通信延迟与同步复杂度。对时间敏感的场景请考虑更严格的时间同步策略。
"""
import os
import sys
import threading
import time
from ctypes import byref, cast, POINTER, c_ubyte

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class ProjectedPixelOverlayNode(Node):
    def __init__(self):
        super().__init__('projected_pixel_overlay_node')

        # ========== 摄像头相关参数（来自原 hikvision 节点） ==========
        self.declare_parameter('mvs_python_path', '/opt/MVS/Samples/64/Python')
        self.declare_parameter('device_index', 0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('grab_timeout_ms', 1000)
        self.declare_parameter('publish_rate_hz', 0.0)
        self.declare_parameter('enable_trigger', False)
        self.declare_parameter('exposure_time_us', 0.0)
        self.declare_parameter('gain', 0.0)

        # ========== 叠加相关参数（来自原 projected_pixel_overlay_node） ==========
        self.declare_parameter('pixel_topic', '/projected_pixel')
        self.declare_parameter('output_topic', '/projected_image')
        self.declare_parameter('draw_radius', 6)
        self.declare_parameter('draw_thickness', 2)
        self.declare_parameter('draw_color_bgr', [0, 0, 255])
        self.declare_parameter('text_color_bgr', [0, 255, 0])
        self.declare_parameter('font_scale', 0.7)
        self.declare_parameter('draw_depth_text', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'Projected Pixel Overlay')
        self.declare_parameter('log_out_of_range', True)
        self.declare_parameter('out_of_range_log_period', 20)
        self.declare_parameter('clamp_pixel', False)

        # ========== 获取参数 ==========
        self.mvs_python_path = str(self.get_parameter('mvs_python_path').value)
        self.device_index = int(self.get_parameter('device_index').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.grab_timeout_ms = int(self.get_parameter('grab_timeout_ms').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.enable_trigger = bool(self.get_parameter('enable_trigger').value)
        self.exposure_time_us = float(self.get_parameter('exposure_time_us').value)
        self.gain = float(self.get_parameter('gain').value)

        self.pixel_topic = str(self.get_parameter('pixel_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.draw_radius = int(self.get_parameter('draw_radius').value)
        self.draw_thickness = int(self.get_parameter('draw_thickness').value)
        self.draw_color_bgr = self._sanitize_color(self.get_parameter('draw_color_bgr').value)
        self.text_color_bgr = self._sanitize_color(self.get_parameter('text_color_bgr').value)
        self.font_scale = float(self.get_parameter('font_scale').value)
        self.draw_depth_text = bool(self.get_parameter('draw_depth_text').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.log_out_of_range = bool(self.get_parameter('log_out_of_range').value)
        self.out_of_range_log_period = int(self.get_parameter('out_of_range_log_period').value)
        self.clamp_pixel = bool(self.get_parameter('clamp_pixel').value)

        # ========== 初始化 SDK ==========
        self._load_mvs_sdk()

        # ========== ROS 话题 ==========
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, self.image_topic, 10)
        self.pub_overlay = self.create_publisher(Image, self.output_topic, 10)
        self.sub_pixel = self.create_subscription(
            PointStamped, self.pixel_topic, self._pixel_callback, 10
        )

        # ========== 内部状态 ==========
        self._running = False
        self._thread = None
        self._cam = None
        self._width = 0
        self._height = 0
        self._bgr_buffer = None
        self._bgr_buffer_size = 0

        # 最新像素（线程安全）
        self._latest_pixel = None
        self._pixel_lock = threading.Lock()
        self._out_of_range_count = 0
        self._invalid_pixel_count = 0

        # 启动相机并采集线程
        self._open_device()
        self._start_grabbing()
        self._start_thread()

        self.get_logger().info('=' * 60)
        self.get_logger().info('Merged ProjectedPixelOverlayNode 已启动')
        self.get_logger().info(f'图像话题: {self.image_topic} -> 发布原始图像')
        self.get_logger().info(f'订阅像素: {self.pixel_topic}')
        self.get_logger().info(f'发布叠加图像: {self.output_topic}')
        self.get_logger().info('=' * 60)

    def _sanitize_color(self, value):
        if isinstance(value, (list, tuple)) and len(value) == 3:
            return [int(max(0, min(255, v))) for v in value]
        return [0, 0, 255]

    def _pixel_callback(self, msg: PointStamped):
        with self._pixel_lock:
            # 只保存最新一条像素信息；时间同步采用帧内最近采样策略
            self._latest_pixel = msg

    def _load_mvs_sdk(self):
        if self.mvs_python_path and self.mvs_python_path not in sys.path:
            sys.path.append(self.mvs_python_path)
        try:
            from MvImport import (  # type: ignore
                MvCamera,
                MV_CC_DEVICE_INFO_LIST,
                MV_CC_DEVICE_INFO,
                MV_GIGE_DEVICE,
                MV_USB_DEVICE,
                MV_ACCESS_Exclusive,
                MV_FRAME_OUT_INFO_EX,
                MVCC_INTVALUE,
            )
        except Exception as exc:
            raise RuntimeError(
                '无法导入海康MVS Python SDK。请确认安装路径并设置 mvs_python_path。'
            ) from exc

        self.MvCamera = MvCamera
        self.MV_CC_DEVICE_INFO_LIST = MV_CC_DEVICE_INFO_LIST
        self.MV_CC_DEVICE_INFO = MV_CC_DEVICE_INFO
        self.MV_GIGE_DEVICE = MV_GIGE_DEVICE
        self.MV_USB_DEVICE = MV_USB_DEVICE
        self.MV_ACCESS_Exclusive = MV_ACCESS_Exclusive
        self.MV_FRAME_OUT_INFO_EX = MV_FRAME_OUT_INFO_EX
        self.MVCC_INTVALUE = MVCC_INTVALUE

    def _open_device(self):
        device_list = self.MV_CC_DEVICE_INFO_LIST()
        tlayer_type = self.MV_GIGE_DEVICE | self.MV_USB_DEVICE
        ret = self.MvCamera.MV_CC_EnumDevices(tlayer_type, device_list)
        if ret != 0:
            raise RuntimeError(f'枚举设备失败: ret=0x{ret:x}')
        if device_list.nDeviceNum <= 0:
            raise RuntimeError('未发现海康相机设备')
        if self.device_index >= device_list.nDeviceNum:
            raise RuntimeError(
                f'设备索引超出范围: index={self.device_index}, device_num={device_list.nDeviceNum}'
            )

        st_device = cast(
            device_list.pDeviceInfo[self.device_index], POINTER(self.MV_CC_DEVICE_INFO)
        ).contents

        self._cam = self.MvCamera()
        ret = self._cam.MV_CC_CreateHandle(st_device)
        if ret != 0:
            raise RuntimeError(f'创建句柄失败: ret=0x{ret:x}')

        ret = self._cam.MV_CC_OpenDevice(self.MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise RuntimeError(f'打开设备失败: ret=0x{ret:x}')

        if st_device.nTLayerType == self.MV_GIGE_DEVICE:
            n_packet_size = self._cam.MV_CC_GetOptimalPacketSize()
            if n_packet_size > 0:
                self._cam.MV_CC_SetIntValue('GevSCPSPacketSize', n_packet_size)

        if self.enable_trigger:
            self._cam.MV_CC_SetEnumValueByString('TriggerMode', 'On')
        else:
            self._cam.MV_CC_SetEnumValueByString('TriggerMode', 'Off')

        if self.exposure_time_us > 0:
            self._cam.MV_CC_SetFloatValue('ExposureTime', self.exposure_time_us)
        if self.gain > 0:
            self._cam.MV_CC_SetFloatValue('Gain', self.gain)

        self._width = self._get_int_value('Width')
        self._height = self._get_int_value('Height')
        if self._width > 0 and self._height > 0:
            self._ensure_bgr_buffer(self._width, self._height)

    def _start_grabbing(self):
        ret = self._cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise RuntimeError(f'开始取流失败: ret=0x{ret:x}')

    def _get_int_value(self, key: str) -> int:
        if not self._cam:
            return 0
        int_value = self.MVCC_INTVALUE()
        ret = self._cam.MV_CC_GetIntValue(key, int_value)
        if ret != 0:
            return 0
        return int(int_value.nCurValue)

    def _ensure_bgr_buffer(self, width: int, height: int):
        size = int(width * height * 3)
        if size <= self._bgr_buffer_size:
            return
        self._bgr_buffer_size = size
        self._bgr_buffer = (c_ubyte * size)()

    def _start_thread(self):
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        frame_info = self.MV_FRAME_OUT_INFO_EX()
        last_publish_time = 0.0

        while rclpy.ok() and self._running:
            if self._bgr_buffer is None and self._width > 0 and self._height > 0:
                self._ensure_bgr_buffer(self._width, self._height)

            if self._bgr_buffer is None:
                time.sleep(0.05)
                continue

            ret = self._cam.MV_CC_GetImageForBGR(
                self._bgr_buffer,
                self._bgr_buffer_size,
                frame_info,
                self.grab_timeout_ms,
            )
            if ret != 0:
                time.sleep(0.005)
                continue

            width = int(getattr(frame_info, 'nWidth', self._width))
            height = int(getattr(frame_info, 'nHeight', self._height))
            if width > 0 and height > 0 and (width != self._width or height != self._height):
                self._width = width
                self._height = height
                self._ensure_bgr_buffer(width, height)

            np_image = np.ctypeslib.as_array(self._bgr_buffer, shape=(self._bgr_buffer_size,))
            np_image = np_image[: width * height * 3].reshape((height, width, 3)).copy()

            now = time.time()
            if self.publish_rate_hz > 0:
                interval = 1.0 / self.publish_rate_hz
                if now - last_publish_time < interval:
                    continue
            last_publish_time = now

            # 发布原始图像（兼容）
            raw_msg = self.bridge.cv2_to_imgmsg(np_image, encoding='bgr8')
            raw_msg.header.stamp = self.get_clock().now().to_msg()
            raw_msg.header.frame_id = self.frame_id
            self.pub_image.publish(raw_msg)

            # 叠加最近的像素并发布叠加图像
            overlay_img = np_image.copy()
            latest = None
            with self._pixel_lock:
                latest = self._latest_pixel

            if latest is not None:
                try:
                    u = float(latest.point.x)
                    v = float(latest.point.y)
                    depth = float(latest.point.z)
                    if np.isfinite([u, v, depth]).all():
                        h, w = overlay_img.shape[:2]
                        if 0 <= u < w and 0 <= v < h:
                            center = (int(round(u)), int(round(v)))
                            cv2.circle(overlay_img, center, self.draw_radius, self.draw_color_bgr, self.draw_thickness)
                            if self.draw_depth_text:
                                text = f'z={depth:.2f}m'
                                text_pos = (center[0] + 8, center[1] - 8)
                                cv2.putText(
                                    overlay_img,
                                    text,
                                    text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    self.font_scale,
                                    self.text_color_bgr,
                                    2,
                                    cv2.LINE_AA,
                                )
                        else:
                            # 像素超出范围，忽略或记录
                            self._out_of_range_count += 1
                            if self.log_out_of_range and self.out_of_range_log_period > 0:
                                if self._out_of_range_count % self.out_of_range_log_period == 1:
                                    self.get_logger().warn(
                                        f'像素超出图像范围: (u={u:.1f}, v={v:.1f}), '
                                        f'图像尺寸: {w}x{h}'
                                    )
                            if self.clamp_pixel and w > 0 and h > 0:
                                cu = int(max(0, min(w - 1, round(u))))
                                cv = int(max(0, min(h - 1, round(v))))
                                center = (cu, cv)
                                cv2.circle(
                                    overlay_img,
                                    center,
                                    self.draw_radius,
                                    self.draw_color_bgr,
                                    self.draw_thickness,
                                )
                    else:
                        self._invalid_pixel_count += 1
                        if self.log_out_of_range and self.out_of_range_log_period > 0:
                            if self._invalid_pixel_count % self.out_of_range_log_period == 1:
                                self.get_logger().warn('收到非有限像素值，已忽略')
                except Exception:
                    pass

            out_msg = self.bridge.cv2_to_imgmsg(overlay_img, encoding='bgr8')
            out_msg.header = raw_msg.header
            self.pub_overlay.publish(out_msg)

            if self.show_window:
                cv2.imshow(self.window_name, overlay_img)
                cv2.waitKey(1)

    def shutdown(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

        if self._cam is not None:
            try:
                self._cam.MV_CC_StopGrabbing()
            except Exception:
                pass
            try:
                self._cam.MV_CC_CloseDevice()
            except Exception:
                pass
            try:
                self._cam.MV_CC_DestroyHandle()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ProjectedPixelOverlayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.show_window:
                cv2.destroyAllWindows()
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
