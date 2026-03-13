#!/usr/bin/env python3

from gi.repository import Gst, GLib
import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import gi
import time

gi.require_version('Gst', '1.0')


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('bitrate', 2000)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('source_codec', 'auto')
        self.declare_parameter('stats_interval_sec', 5.0)
        self.declare_parameter('no_frame_warn_sec', 3.0)

        device = self.get_parameter(
            'device').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter(
            'height').get_parameter_value().integer_value
        framerate = self.get_parameter(
            'framerate').get_parameter_value().integer_value
        bitrate = self.get_parameter(
            'bitrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        source_codec = self.get_parameter(
            'source_codec').get_parameter_value().string_value.lower()
        self.stats_interval_sec = self.get_parameter(
            'stats_interval_sec').get_parameter_value().double_value
        self.no_frame_warn_sec = self.get_parameter(
            'no_frame_warn_sec').get_parameter_value().double_value

        if source_codec == 'auto':
            source_codec = 'h264' if (
                'Huawei' in device or 'HiCamera' in device) else 'mjpeg'

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher = self.create_publisher(
            CompressedVideo, 'compressed_video', qos_profile)

        self.device = device
        self.source_codec = source_codec
        self.frame_count = 0
        self.last_stats_frame_count = 0
        self.last_stats_time = time.monotonic()
        self.last_frame_time = None
        self.logged_first_frame = False
        self.no_frame_warned = False

        Gst.init(None)

        if source_codec == 'h264':
            pipeline_str = (
                f"v4l2src device={device} ! "
                f"video/x-h264,width={width},height={height},framerate={framerate}/1 ! "
                "h264parse config-interval=1 ! "
                "video/x-h264,stream-format=byte-stream,alignment=au ! "
                "appsink name=sink emit-signals=True sync=False"
            )
        elif source_codec == 'mjpeg':
            pipeline_str = (
                f"v4l2src device={device} ! "
                f"image/jpeg,width={width},height={height},framerate={framerate}/1 ! "
                "jpegdec ! videoconvert ! queue ! "
                f"x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast key-int-max={framerate} ! "
                "video/x-h264,profile=baseline ! "
                "h264parse config-interval=1 ! "
                "video/x-h264,stream-format=byte-stream,alignment=au ! "
                "appsink name=sink emit-signals=True sync=False"
            )
        else:
            self.get_logger().warn(
                f"Unknown source_codec '{source_codec}', fallback to mjpeg"
            )
            pipeline_str = (
                f"v4l2src device={device} ! "
                f"image/jpeg,width={width},height={height},framerate={framerate}/1 ! "
                "jpegdec ! videoconvert ! queue ! "
                f"x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast key-int-max={framerate} ! "
                "video/x-h264,profile=baseline ! "
                "h264parse config-interval=1 ! "
                "video/x-h264,stream-format=byte-stream,alignment=au ! "
                "appsink name=sink emit-signals=True sync=False"
            )

        self.get_logger().info(f"Starting pipeline: {pipeline_str}")
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.bus = self.pipeline.get_bus()

        sink = self.pipeline.get_by_name('sink')
        if sink is None:
            self.get_logger().error('appsink named "sink" was not found in pipeline')
            raise RuntimeError('appsink is missing in camera pipeline')
        sink.connect('new-sample', self.on_new_sample)

        state_ret = self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(
            f"camera config: device={self.device}, codec={self.source_codec}, "
            f"size={width}x{height}, fps={framerate}, bitrate={bitrate}, state_ret={state_ret.value_nick}"
        )

        self.bus_timer = self.create_timer(0.2, self.poll_gst_bus)
        self.stats_timer = self.create_timer(self.stats_interval_sec, self.log_stats)

    def poll_gst_bus(self):
        while True:
            message = self.bus.timed_pop_filtered(
                0,
                Gst.MessageType.ERROR
                | Gst.MessageType.WARNING
                | Gst.MessageType.EOS
                | Gst.MessageType.STATE_CHANGED
            )

            if message is None:
                break

            if message.type == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                self.get_logger().error(
                    f"GStreamer ERROR from {message.src.get_name()}: {err}; debug={debug}"
                )
            elif message.type == Gst.MessageType.WARNING:
                warn, debug = message.parse_warning()
                self.get_logger().warn(
                    f"GStreamer WARNING from {message.src.get_name()}: {warn}; debug={debug}"
                )
            elif message.type == Gst.MessageType.EOS:
                self.get_logger().error('GStreamer EOS received (stream ended)')
            elif message.type == Gst.MessageType.STATE_CHANGED and message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                self.get_logger().info(
                    f"Pipeline state changed: {old_state.value_nick} -> "
                    f"{new_state.value_nick} (pending={pending_state.value_nick})"
                )

    def log_stats(self):
        now = time.monotonic()
        elapsed = now - self.last_stats_time
        frames_delta = self.frame_count - self.last_stats_frame_count
        fps = (frames_delta / elapsed) if elapsed > 0 else 0.0

        self.get_logger().info(
            f"video stats: device={self.device}, frame_id={self.frame_id}, "
            f"topic=compressed_video, frames={self.frame_count}, fps={fps:.2f}"
        )

        if self.last_frame_time is None:
            self.get_logger().warn(
                f"no frame received yet: device={self.device}, codec={self.source_codec}"
            )
        else:
            no_frame_for = now - self.last_frame_time
            if no_frame_for > self.no_frame_warn_sec:
                if not self.no_frame_warned:
                    self.get_logger().warn(
                        f"frame stall detected: no frame for {no_frame_for:.2f}s "
                        f"(threshold={self.no_frame_warn_sec:.2f}s, device={self.device})"
                    )
                    self.no_frame_warned = True
            else:
                self.no_frame_warned = False

        self.last_stats_time = now
        self.last_stats_frame_count = self.frame_count

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            self.get_logger().warn('pull-sample returned None')
            return Gst.FlowReturn.ERROR
        buf = sample.get_buffer()

        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            msg = CompressedVideo()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.frame_id = self.frame_id
            msg.data = map_info.data
            msg.format = "h264"

            self.publisher.publish(msg)
            self.frame_count += 1
            self.last_frame_time = time.monotonic()
            if not self.logged_first_frame:
                self.get_logger().info(
                    f"first frame published: frame_id={self.frame_id}, bytes={len(map_info.data)}"
                )
                self.logged_first_frame = True
            buf.unmap(map_info)
        else:
            self.get_logger().warn('failed to map Gst buffer for read')

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
