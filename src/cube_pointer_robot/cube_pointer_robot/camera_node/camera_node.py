#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import defaultdict

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        
        # Tunable parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', '/camera/image_raw'),
                ('min_contour_area', 300),
                ('max_contour_area', 5000),
                ('aspect_ratio_range', [0.7, 1.3]),
                ('solidity_threshold', 0.8),
                ('publish_markers', True),
                ('debug_view', False),
                ('camera_frame', 'camera_color_optical_frame'),
                ('blur_kernel_size', 5),
                ('morphology_kernel_size', 5),
                ('red_lower1', [0, 100, 100]),
                ('red_upper1', [10, 255, 255]),
                ('red_lower2', [170, 100, 100]),
                ('red_upper2', [180, 255, 255]),
                ('blue_lower', [90, 50, 50]),
                ('blue_upper', [130, 255, 255]),
                ('yellow_lower', [20, 100, 100]),
                ('yellow_upper', [40, 255, 255]),
            ]
        )
        
        self.bridge = CvBridge()
        self.detected_colors = defaultdict(list)
        self.marker_id_counter = 0
        
        # Publishers
        self.detection_pub = self.create_publisher(PointStamped, '/detected_cubes', 10)
        self.marker_pub    = self.create_publisher(MarkerArray,    '/visualization_markers', 10)
        
        # Subscriber
        self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            qos_profile=10
        )
        
        self.get_logger().info("Color detection node initialized and ready")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Preprocess
        blur_k = self.get_parameter('blur_kernel_size').value
        blurred = cv2.medianBlur(cv_image, blur_k)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Load color ranges
        colors = {
            'red': {
                'lower1': np.array(self.get_parameter('red_lower1').value),
                'upper1': np.array(self.get_parameter('red_upper1').value),
                'lower2': np.array(self.get_parameter('red_lower2').value),
                'upper2': np.array(self.get_parameter('red_upper2').value),
            },
            'blue': {
                'lower': np.array(self.get_parameter('blue_lower').value),
                'upper': np.array(self.get_parameter('blue_upper').value),
            },
            'yellow': {
                'lower': np.array(self.get_parameter('yellow_lower').value),
                'upper': np.array(self.get_parameter('yellow_upper').value),
            }
        }

        # Clear last detections
        self.detected_colors.clear()
        
        # Process each color
        for color, rng in colors.items():
            if color == 'red':
                m1 = cv2.inRange(hsv, rng['lower1'], rng['upper1'])
                m2 = cv2.inRange(hsv, rng['lower2'], rng['upper2'])
                mask = cv2.bitwise_or(m1, m2)
            else:
                mask = cv2.inRange(hsv, rng['lower'], rng['upper'])
            
            # Morphology
            mk = self.get_parameter('morphology_kernel_size').value
            kernel = np.ones((mk, mk), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                mn   = self.get_parameter('min_contour_area').value
                mx   = self.get_parameter('max_contour_area').value
                if not (mn < area < mx):
                    continue
                
                # Shape checks
                hull      = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                solidity  = area / hull_area if hull_area > 0 else 0
                x, y, w, h= cv2.boundingRect(cnt)
                ar        = float(w)/h
                ar_rng    = self.get_parameter('aspect_ratio_range').value
                if solidity < self.get_parameter('solidity_threshold').value or not (ar_rng[0] < ar < ar_rng[1]):
                    continue
                
                # Compute center
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                self.detected_colors[color].append((cX, cY, w, h))
                
                # Publish detection
                pt = PointStamped()
                pt.header      = msg.header
                pt.point.x     = float(cX)
                pt.point.y     = float(cY)
                pt.point.z     = float(w)  # store width in z
                self.detection_pub.publish(pt)

        # Publish markers if desired
        if self.get_parameter('publish_markers').value:
            self._publish_markers(msg.header)

    def _publish_markers(self, header):
        ma = MarkerArray()
        # clear old
        clr = Marker()
        clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        # new markers
        for color, cubes in self.detected_colors.items():
            for _, (x, y, w, h) in enumerate(cubes):
                mk = Marker()
                mk.header          = header
                mk.ns              = color
                mk.id              = self.marker_id_counter
                self.marker_id_counter += 1
                mk.type            = Marker.CUBE
                mk.action          = Marker.ADD
                mk.pose.position.x = float(x)
                mk.pose.position.y = float(y)
                mk.pose.position.z = 0.0
                mk.pose.orientation.w = 1.0
                mk.scale.x = float(w)
                mk.scale.y = float(h)
                mk.scale.z = 0.01
                if   color=='red':    mk.color.r, mk.color.g, mk.color.b = 1.0,0.0,0.0
                elif color=='blue':   mk.color.r, mk.color.g, mk.color.b = 0.0,0.0,1.0
                else:                 mk.color.r, mk.color.g, mk.color.b = 1.0,1.0,0.0
                mk.color.a        = 0.7
                mk.lifetime.nanosec = int(2e8)
                ma.markers.append(mk)
        self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down color detection node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
