#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from aruco_opencv_msgs.msg import ArucoDetection
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

class MarkerPoseEstimator(Node):
    def __init__(self):
        super().__init__('marker_pose_estimator')
        
        # パラメータの宣言
        self.declare_parameter('camera_info_topic', '/rgb/camera_info')
        self.declare_parameter('depth_topic', '/depth_to_rgb/image_raw')
        self.declare_parameter('markers_topic', '/aruco_markers')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('marker_size', 0.20)
        
        # トピック名の取得
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.markers_topic = self.get_parameter('markers_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.marker_size = self.get_parameter('marker_size').value
        
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None
        
        # TFバッファとリスナー
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # サブスクライバー
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10)
        self.markers_sub = self.create_subscription(
            ArucoDetection, self.markers_topic, self.markers_callback, 10)
            
        # パブリッシャー
        self.marker_point_pub = self.create_publisher(PointStamped, 'marker_position', 10)
        self.viz_pub = self.create_publisher(MarkerArray, 'marker_visualization', 10)
        
        self.get_logger().info('MarkerPoseEstimator initialized with ROI refinement')

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

    def markers_callback(self, msg):
        if self.camera_info is None or self.depth_image is None:
            return

        marker_array = MarkerArray()
        
        for idx, marker in enumerate(msg.markers):
            # 3D位置からの逆投影によりROIを算出
            x_m = marker.pose.position.x
            y_m = marker.pose.position.y
            z_m = marker.pose.position.z
            
            if z_m <= 0:
                continue
                
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            
            u_center = (x_m * fx / z_m) + cx
            v_center = (y_m * fy / z_m) + cy
            
            # マーカーサイズ (0.2m) に基づくROIサイズ (ピクセル)
            # 余裕を持って1.5倍の領域を探索
            pixel_size = (self.marker_size * fx / z_m) * 1.5
            
            x_min = int(u_center - pixel_size / 2)
            x_max = int(u_center + pixel_size / 2)
            y_min = int(v_center - pixel_size / 2)
            y_max = int(v_center + pixel_size / 2)
            
            # 画像範囲内にクリップ
            h, w = self.depth_image.shape
            x_min, x_max = max(0, x_min), min(w - 1, x_max)
            y_min, y_max = max(0, y_min), min(h - 1, y_max)
            
            if x_max <= x_min or y_max <= y_min:
                continue
                
            # ROI内のデプス値を取得
            roi_depth = self.depth_image[y_min:y_max, x_min:x_max]
            
            # 無効値 (0) を除外
            valid_depths = roi_depth[roi_depth > 0]
            
            if len(valid_depths) < 5:
                # デプスが取れない場合はAruco自体の推定値を使用
                estimated_depth = z_m
            else:
                # メディアンフィルタと統計的除去
                median_depth = np.median(valid_depths)
                std_depth = np.std(valid_depths)
                sane_depths = valid_depths[np.abs(valid_depths - median_depth) <= std_depth]
                
                if len(sane_depths) == 0:
                    estimated_depth = median_depth
                else:
                    estimated_depth = np.mean(sane_depths)
                
            # デプス値の単位変換 (Azure Kinect 16UC1=mm)
            if self.depth_image.dtype == np.uint16:
                z = float(estimated_depth) / 1000.0
            else:
                z = float(estimated_depth)

            # 補正後の3D座標
            x_cam = (u_center - cx) * z / fx
            y_cam = (v_center - cy) * z / fy
            z_cam = z
            
            # PointStampedの作成
            p_cam = PointStamped()
            p_cam.header.frame_id = msg.header.frame_id
            p_cam.header.stamp = msg.header.stamp
            p_cam.point = Point(x=x_cam, y=y_cam, z=z_cam)
            
            # base_link座標系への変換
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    p_cam.header.frame_id,
                    rclpy.time.Time()
                )
                p_base = tf2_geometry_msgs.do_transform_point(p_cam, transform)
                
                # パブリッシュ
                self.marker_point_pub.publish(p_base)
                
                # 可視化
                viz_marker = self.create_viz_marker(p_base, int(marker.marker_id), idx)
                marker_array.markers.append(viz_marker)
                
                self.get_logger().info(
                    f'Marker ID {marker.marker_id}: x={p_base.point.x:.2f}, y={p_base.point.y:.2f}, z={p_base.point.z:.2f} (refined)'
                )
                
            except TransformException as e:
                self.get_logger().warn(f'TF Transform error: {e}')
                
        if marker_array.markers:
            self.viz_pub.publish(marker_array)

    def create_viz_marker(self, p_stamped, marker_id, idx):
        m = Marker()
        m.header = p_stamped.header
        m.ns = "aruco_refined"
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = p_stamped.point
        m.pose.orientation.w = 1.0
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        return m

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
