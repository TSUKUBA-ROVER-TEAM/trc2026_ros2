#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # パラメータの宣言
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('bitrate', 2000)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('encoding', 'h264') # 現時点ではh264のみサポート想定
        
        device = self.get_parameter('device').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # QoS設定: 映像配信に適したBest Effortを使用
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher = self.create_publisher(CompressedVideo, 'compressed_video', qos_profile)
        
        # GStreamerの初期化
        Gst.init(None)
        
        # カメラの特性に合わせてパイプラインを構築
        # Huawei HiCameraなどは native H.264を吐くが、汎用性を考慮して
        # 一旦rawにデコードしてからx264encで叩き直す(webrtc_auto_launch.shの構成を参考)
        
        if "Huawei" in device or "HiCamera" in device:
             # HiCamera用 (H.264 native)
            pipeline_str = (
                f"v4l2src device={device} ! "
                f"video/x-h264,width={width},height={height},framerate={framerate}/1 ! "
                "h264parse config-interval=1 ! "
                "appsink name=sink emit-signals=True sync=False"
            )
        else:
            # 一般的なカメラ用 (MJPEG/RAW -> H.264)
            pipeline_str = (
                f"v4l2src device={device} ! "
                "decodebin ! videoconvert ! "
                f"x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast key-int-max={framerate} ! "
                "video/x-h264,profile=baseline ! "
                "h264parse config-interval=1 ! "
                "appsink name=sink emit-signals=True sync=False"
            )
            
        self.get_logger().info(f"Starting pipeline: {pipeline_str}")
        self.pipeline = Gst.parse_launch(pipeline_str)
        
        sink = self.pipeline.get_by_name('sink')
        sink.connect('new-sample', self.on_new_sample)
        
        self.pipeline.set_state(Gst.State.PLAYING)

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        
        # データの取得
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            data = map_info.data
            
            # CompressedVideoメッセージの作成
            msg = CompressedVideo()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.frame_id = self.frame_id
            msg.data = data
            msg.format = "h264"
            
            self.publisher.publish(msg)
            buf.unmap(map_info)
            
        return Gst.FlowReturn.OK

    def __del__(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.set_state(Gst.State.NULL)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
