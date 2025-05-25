import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transitions import Machine


class Coordinator(Node):
    states = ['idle', 'home', 'search', 'point', 'missing_cube', 'error']

    def __init__(self):
        super().__init__('coordinator_node')

        # Internal state variables
        self.latest_arm_value = [0, 0, 0]
        self.latest_box_value = None

        self.home_position = [0, 0, 0]
        self.red_box_position = None
        self.blue_box_position = None
        self.green_box_position = None
        self.all_boxes_found = False

        self.search_positions = [
            [0.1, 0.1, 0.3],
            [0.2, 0.1, 0.3],
            [0.3, 0.1, 0.3],
            [0.3, 0.2, 0.3],
            [0.2, 0.2, 0.3],
            [0.1, 0.2, 0.3],
        ]
        self.search_index = 0
        self.searching = False

        self.pointing_targets = []
        self.current_point_index = 0
        self.pointing_in_progress = False

        self.missing_cube_position = [0.0, 0.0, 0.1]
        self.waiting_in_missing_position = False

        # Publishers and Subscribers
        self.command_pub = self.create_publisher(String, '/command_topic', 10)
        self.create_subscription(String, '/box_coordinates', self.box_callback, 10)
        self.create_subscription(String, '/arm_coordinates', self.arm_callback, 10)

        # State machine setup
        transitions = [
            {'trigger': 'move_home', 'source': 'idle', 'dest': 'home'},
            {'trigger': 'start_searching', 'source': 'home', 'dest': 'search'},
            {'trigger': 'start_pointing', 'source': 'search', 'dest': 'point'},
            {'trigger': 'cube_not_found', 'source': 'search', 'dest': 'missing_cube'},
            {'trigger': 'pointing_complete', 'source': 'point', 'dest': 'home'},
            {'trigger': 'reset', 'source': '*', 'dest': 'idle'},
            {'trigger': 'error_occurred', 'source': '*', 'dest': 'error'}
        ]

        self.machine = Machine(model=self, states=Coordinator.states, transitions=transitions, initial='idle')

        self.get_logger().info("Coordinator node initialized. Current state: 'idle'")

    def box_callback(self, msg):
        self.latest_box_value = msg.data
        self.get_logger().info(f"[Box] Received: {self.latest_box_value}")
        self.evaluate_state()

    def arm_callback(self, msg):
        try:
            self.latest_arm_value = list(map(float, msg.data.strip("[]").split(',')))
            self.get_logger().info(f"[Arm] Received: {self.latest_arm_value}")
            self.evaluate_state()
        except Exception as e:
            self.get_logger().error(f"Error parsing arm coordinates: {e}")

    def evaluate_state(self):
        state = self.state
        arm = self.latest_arm_value
        boxes = self.latest_box_value

        self.get_logger().info(f"[Evaluate] State: {state}, Arm: {arm}, Box: {boxes}")

        if state == 'idle':
            self.get_logger().info("Moving to home state.")
            self.move_home()
            self.send_command("MOVE_HOME")

        elif state == 'home':
            if arm == self.home_position:
                self.get_logger().info("At home position. Starting search.")
                self.start_searching()
                self.send_command("SEARCH")

        elif state == 'search':
            self.check_boxes(boxes)

            if self.all_boxes_found:
                self.get_logger().info("All boxes found. Starting to point.")
                self.start_pointing()
                self.send_command("POINT")
                self.searching = False
                self.search_index = 0
            else:
                if not self.searching:
                    self.get_logger().info("Starting box search sequence.")
                    self.searching = True
                    self.send_command(f"MOVE_TO {self.search_positions[self.search_index]}")
                elif arm == self.search_positions[self.search_index]:
                    self.search_index += 1
                    if self.search_index < len(self.search_positions):
                        self.send_command(f"MOVE_TO {self.search_positions[self.search_index]}")
                    else:
                        self.get_logger().warn("Cube not found after search. Going to missing_cube.")
                        self.cube_not_found()
                        self.send_command("CUBE_MISSING")
                        self.searching = False
                        self.search_index = 0

        elif state == 'point':
            if not self.pointing_in_progress:
                self.pointing_targets = [
                    self.red_box_position,
                    self.green_box_position,
                    self.blue_box_position
                ]
                self.current_point_index = 0
                self.pointing_in_progress = True
                self.send_command(f"MOVE_TO {self.pointing_targets[self.current_point_index]}")
                self.get_logger().info("Starting pointing sequence.")
            else:
                if arm == self.pointing_targets[self.current_point_index]:
                    self.current_point_index += 1
                    if self.current_point_index < len(self.pointing_targets):
                        self.send_command(f"MOVE_TO {self.pointing_targets[self.current_point_index]}")
                    else:
                        self.get_logger().info("Finished pointing to all boxes. Returning home.")
                        self.pointing_complete()
                        self.send_command("MOVE_HOME")
                        self.pointing_in_progress = False

        elif state == 'missing_cube':
            if not self.waiting_in_missing_position:
                self.get_logger().info("Moving to missing cube position.")
                self.send_command(f"MOVE_TO {self.missing_cube_position}")
                self.waiting_in_missing_position = True
            elif arm == self.missing_cube_position:
                self.get_logger().info("Reached missing cube position. Resetting to idle.")
                self.waiting_in_missing_position = False
                self.reset()
                self.send_command("RESET")

        elif state == 'error':
            self.get_logger().error("Error state reached. Awaiting manual reset.")

    def check_boxes(self, box_msg):
        if not box_msg or self.all_boxes_found:
            return

        try:
            boxes = box_msg.split(',')
            for box in boxes:
                parts = box.split(':')
                if len(parts) != 2:
                    continue
                color, coords = parts
                x_y = coords.split(',')
                if len(x_y) != 2:
                    continue
                x, y = map(float, x_y)

                if color == 'red' and self.red_box_position is None:
                    self.red_box_position = self.convert_to_global_coordinates(x, y)
                elif color == 'blue' and self.blue_box_position is None:
                    self.blue_box_position = self.convert_to_global_coordinates(x, y)
                elif color == 'green' and self.green_box_position is None:
                    self.green_box_position = self.convert_to_global_coordinates(x, y)

            if self.check_all_boxes_found():
                self.all_boxes_found = True
                self.get_logger().info("All boxes found and positions calculated.")

        except Exception as e:
            self.get_logger().error(f"Error parsing box message: {e}")

    def convert_to_global_coordinates(self, x_local, y_local):
        arm_x, arm_y, arm_z = self.latest_arm_value
        global_x = arm_x + x_local
        global_y = arm_y + y_local
        global_z = arm_z
        return [global_x, global_y, global_z]

    def send_command(self, command_str):
        msg = String()
        msg.data = command_str
        self.command_pub.publish(msg)
        self.get_logger().info(f"[Command] Sent: {command_str}")

    def check_all_boxes_found(self):
        return (self.red_box_position is not None and
                self.blue_box_position is not None and
                self.green_box_position is not None)


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down coordinator.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
