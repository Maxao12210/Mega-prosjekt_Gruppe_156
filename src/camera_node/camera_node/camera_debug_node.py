#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MaskDebugNode(Node):
    def __init__(self):
        super().__init__('mask_debug_node')
        # Subscribe to image topic
        self.declare_parameter('camera_topic', '/image_raw')
        topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.create_subscription(Image, topic, self.image_callback, 10)
        self.bridge = CvBridge()

        # Define windows
        cv2.namedWindow("Detected Colors", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Red Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Blue Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Green Mask", cv2.WINDOW_NORMAL)

        # HSV ranges
        self.color_ranges = {
            'Red':   ((170, 100, 100), (180, 255, 255)),
            'Blue':  ((100, 150, 0),   (140, 255, 255)),
            'Green': ((40, 50, 50),    (80, 255, 255))
        }
        self.min_area = 500
        self.get_logger().info(f"MaskDebugNode subscribed to {topic}")

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Conversion error: {e}")
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        output = img.copy()

        for name, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) >= self.min_area:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(output, name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    break

            cv2.imshow(f"{name} Mask", mask)

        cv2.imshow("Detected Colors", output)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MaskDebugNode()
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
