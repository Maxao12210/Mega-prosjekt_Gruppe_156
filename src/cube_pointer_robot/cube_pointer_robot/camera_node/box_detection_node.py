#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare the camera topic parameter
        self.declare_parameter('camera_topic', '/image_raw')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f"Subscribed to {camera_topic}")

        # Publisher for box coordinates
        self.coord_pub = self.create_publisher(
            String,
            '/box_coordinates',
            10
        )
        self.get_logger().info("Publishing box coordinates on /box_coordinates")

        # Prepare OpenCV window
        cv2.namedWindow("Detected Colors", cv2.WINDOW_NORMAL)

        # Define HSV ranges in the order to search
        self.color_ranges = [
            ('Red', (170, 100, 100), (180, 255, 255)),
            ('Blue', (100, 150, 0), (140, 255, 255)),
            ('Green', (40, 50, 50), (80, 255, 255))  # Replaced Yellow with Green
        ]

        # Minimum contour area
        self.min_area = 500

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        coord_parts = []

        for name, lower, upper in self.color_ranges:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            found = False
            for cnt in contours:
                if cv2.contourArea(cnt) >= self.min_area:
                    x, y, w, h = cv2.boundingRect(cnt)
                    u = x + w // 2
                    v = y + h // 2
                    label = f"{name} ({u},{v})"
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    coord_parts.append(f"{name}:{u},{v}")
                    found = True
                    break
            if not found:
                coord_parts.append(f"{name}:NA")

        coord_str = '; '.join(coord_parts)
        self.coord_pub.publish(String(data=coord_str))
        self.get_logger().info(coord_str)

        cv2.imshow("Detected Colors", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()