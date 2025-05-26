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

        # ---- PARAMETERS ----
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('min_area', 500)              # filter small contours
        self.declare_parameter('aspect_ratio_thresh', 0.2)   # allowed deviation from square

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        ar_thresh = self.get_parameter('aspect_ratio_thresh').get_parameter_value().double_value

        # accept aspect ratios in [1 - ar_thresh, 1 + ar_thresh]
        self.min_ar = 1.0 - ar_thresh
        self.max_ar = 1.0 + ar_thresh

        # Defines the HSV color ranges for red, blue and green.

        self.color_ranges = [

            ('Red',   (170, 100, 100), (180, 255, 255)),
            ('Blue',  (100, 150,   0), (140, 255, 255)),
            ('Green', ( 40,  50,  50), ( 80, 255, 255)),

        ]

        # ---- SETUP ----
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )
        self.coord_pub = self.create_publisher(String, '/box_coordinates', 10)

        cv2.namedWindow("Detected Colors", cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Subscribed to {camera_topic}, publishing /box_coordinates")

    def image_callback(self, msg: Image):
        # 1) Converts the raw image to an openCV image.
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        #Converts the default openCV color format of the image from BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #empty list to store the coordiantes of the cubes.
        coord_parts = []

        # 2) Loop through each color range
        for name, lower, upper in self.color_ranges:

            #Creates a binary mask for each color range where all pixels not at the defined color (red,blue or green depending on which color is being processed) are black, and the pixels with the correct color are white.
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            #Finds the edges of all white objects within the mask.
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            #Flag for when a cube has been found.
            found = False
            for cnt in contours:

                # Calculates the area of each contour.
                area = cv2.contourArea(cnt)

                #If the current object contour area is below the minimum area the loop skips the current contour
                if area < self.min_area:
                    continue



                #finds the x and y coordinates of the top left corner of the contour, and calculates the width and height of the rectangle.
                x, y, w, h = cv2.boundingRect(cnt)

                #calculates aspect ratio of the rectanblge
                ar = float(w) / float(h) if h else 0.0

                # shape approximation to count corners by using arcLength to calculate the perimeter.
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)

                # only accept roughly-square 4-corner shapes
                if self.min_ar <= ar <= self.max_ar and len(approx) == 4:

                    #u is the x coordinate of the center of the rectangle/cube
                    u = x + w // 2
                    #v is the y coordinate of the center of the rectangle/cube.
                    v = y + h // 2

                    # draws a yellow rectangle onto the openCV image.
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)

                    # Creates a label for the rectangle
                    cv2.putText(
                        img,
                        f"{name} ({u},{v})",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2
                    )

                    #Appends the coordinate of the found cube to the coord_parts list with name and x,y coordinate of the center of the rectangle.
                    coord_parts.append(f"{name}:{u},{v}")

                    #Sets the flag true so the code stops searching for the cube.
                    found = True
                    break

            #If the cube is not found after all contours have been searched then the coordinate is written as NA.
            if not found:
                coord_parts.append(f"{name}:NA")

        # 3) Publish and display
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
