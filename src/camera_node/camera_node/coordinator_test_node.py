#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transitions import Machine

class TestCoordinator(Node):
    # States: idle -> search -> point -> idle
    states = ['idle', 'search', 'point']

    def __init__(self):
        super().__init__('test_coordinator')

        # Box positions storage
        self.red_box = None
        self.blue_box = None
        self.green_box = None

        # Publishers and subscribers
        self.command_pub = self.create_publisher(String, '/test_command', 10)
        # Publisher for all_found status
        self.status_pub = self.create_publisher(String, '/all_boxes_found', 10)
        # Publisher for current state
        self.state_pub = self.create_publisher(String, '/current_state', 10)
        self.create_subscription(String, '/box_coordinates', self.box_callback, 10)

        # State machine transitions
        transitions = [
            {'trigger': 'start_search', 'source': 'idle', 'dest': 'search', 'after': 'publish_state'},
            {'trigger': 'all_found',    'source': 'search', 'dest': 'point',  'after': 'publish_state'},
            {'trigger': 'reset',        'source': 'point',  'dest': 'idle',   'after': 'publish_state'},
        ]

        self.machine = Machine(model=self, states=TestCoordinator.states,
                               transitions=transitions, initial='idle')

        self.get_logger().info("TestCoordinator initialized in 'idle'.")
        # Publish initial state and status
        self.publish_state()
        self.publish_status(False)

        # Kick off search immediately
        self.start_search()
        self.send_command('SEARCH')

    def box_callback(self, msg: String):
        parts = msg.data.split(';')
        for p in parts:
            p = p.strip()  # remove whitespace
            try:
                color, coords = p.split(':')
                x, y = map(float, coords.strip('()').split(','))
            except Exception:
                continue

            c = color.strip().lower()
            if c == 'red' and self.red_box is None:
                self.red_box = (x, y)
            elif c == 'blue' and self.blue_box is None:
                self.blue_box = (x, y)
            elif c == 'green' and self.green_box is None:
                self.green_box = (x, y)

        self.get_logger().info(f"Boxes: R={self.red_box}, B={self.blue_box}, G={self.green_box}")

        # Check if all found and publish status
        all_found = None not in (self.red_box, self.blue_box, self.green_box)
        self.publish_status(all_found)

        if self.state == 'search' and all_found:
            self.get_logger().info("All boxes detected, transitioning to 'point'.")
            self.all_found()
            self.send_command('POINT')
            # After pointing, reset to idle for next test
            self.reset()
            self.send_command('RESET')

    def send_command(self, cmd: str):
        msg = String(data=cmd)
        self.command_pub.publish(msg)
        self.get_logger().info(f"[Command] {cmd}")

    def publish_status(self, status: bool):
        """Publish whether all boxes have been found: 'YES' or 'NO'."""
        msg = String(data='YES' if status else 'NO')
        self.status_pub.publish(msg)
        self.get_logger().info(f"[Status] All boxes found: {msg.data}")

    def publish_state(self):
        """Publish the current state of the coordinator."""
        msg = String(data=self.state)
        self.state_pub.publish(msg)
        self.get_logger().info(f"[State] Current state: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = TestCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
