#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Deklarer parameter for kamera-topic
        self.declare_parameter('camera_topic', '/camera/image_raw')

        self.bridge = CvBridge()

        # Hent topic fra parameter
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        # Abonner på kamera-topic
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Subscribed to {camera_topic}")

    def image_callback(self, msg):
        try:
            # Konverter ROS-bilde til OpenCV-format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # HSV-fargeområder
        color_ranges = {
            'Red': ((0, 120, 70), (10, 255, 255)),
            'Blue': ((100, 150, 0), (140, 255, 255)),
            'Green': ((40, 70, 70), (90, 255, 255)),
        }

        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(cv_image, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Detected Colors", cv_image)
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
