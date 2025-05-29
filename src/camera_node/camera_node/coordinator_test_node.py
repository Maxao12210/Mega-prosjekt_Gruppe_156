#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from object_reference_msg.srv import ObjectReference

class TestCoordinator(Node):
    def __init__(self):
        super().__init__('test_coordinator')

        # Known TCP tool position in world frame (meters)
        self.tcp_x = 0.37
        self.tcp_y = 0.110
        self.tcp_z = 0.418  # not used here, but available
        self.cam_off_x = 0.07
        self.cam_off_y = 0.10

        # Camera resolution and calibration
        self.img_w = 1280
        self.img_h = 720
        self.mpp_x = 0.0005  # meters per pixel in X
        self.mpp_y = 0.0005  # meters per pixel in Y

        # placeholders for first-seen pixel coords
        self.red_px = None
        self.blue_px = None
        self.green_px = None

        # subscribe to box detections
        self.create_subscription(
            String, '/box_coordinates',
            self.box_callback, 10)

        # publisher for global coords
        self.coords_pub = self.create_publisher(
            String, '/tcp_global_coords', 10)

        # set up service client
        self.cli = self.create_client(ObjectReference, 'get_tcp_pos')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service get_tcp_pos not available, shutting down')
            rclpy.shutdown()

    def box_callback(self, msg: String):
        # parse "red:(px,py); blue:(px,py); green:(px,py)"
        for part in msg.data.split(';'):
            try:
                color, coords = part.strip().split(':')
                px, py = map(int, coords.strip('()').split(','))
            except ValueError:
                continue
            c = color.strip().lower()
            if c == 'red'   and self.red_px   is None: self.red_px   = (px, py)
            if c == 'blue'  and self.blue_px  is None: self.blue_px  = (px, py)
            if c == 'green' and self.green_px is None: self.green_px = (px, py)

        # once we have all three, call the service and then exit
        if None not in (self.red_px, self.blue_px, self.green_px):
            self.call_service_and_exit()


    def pixel_to_world(self, px, py):
        # 1) project pixel offset into meters (centered at optical axis)
        dx = (px - self.img_w/2) * self.mpp_x
        dy = (py - self.img_h/2) * self.mpp_y

        # 2) camera origin in world = TCP + camera offset
        cam_x = self.tcp_x + self.cam_off_x
        cam_y = self.tcp_y + self.cam_off_y
        # cam_z = self.tcp_z + self.cam_off_z  # if you later need Z

        # 3) world coords = camera_origin + projection
        return cam_x + dx, cam_y + dy

    def call_service_and_exit(self):
        # compute each global (x,y)
        rx, ry = self.pixel_to_world(*self.red_px)
        bx, by = self.pixel_to_world(*self.blue_px)
        gx, gy = self.pixel_to_world(*self.green_px)

        # publish the same global coords as a string
        coords_msg = String()
        coords_msg.data = (
            f"RED:({rx:.3f},{ry:.3f}); "
            f"GREEN:({gx:.3f},{gy:.3f}); "
            f"BLUE:({bx:.3f},{by:.3f})"
        )
        self.coords_pub.publish(coords_msg)
        self.get_logger().info(f"Published global coords → {coords_msg.data}")

        # now build and send the service request
        req = ObjectReference.Request()
        req.x1, req.y1 = rx, ry
        req.x2, req.y2 = gx, gy
        req.x3, req.y3 = bx, by

        self.get_logger().info(f"Calling get_tcp_pos → {coords_msg.data}")

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res and res.success:
            self.get_logger().info('Service returned success=True')
        else:
            self.get_logger().error('Service returned success=False or failed')

        rclpy.shutdown()  # done, exit

def main(args=None):
    rclpy.init(args=args)
    node = TestCoordinator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
