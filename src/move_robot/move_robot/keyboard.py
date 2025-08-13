import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from time import sleep
import re

class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")

        self.toggle_log_pub = self.create_publisher(Bool, "/kb/toggle_log", 10)
        self.shutdown_pub = self.create_publisher(Bool, "/kb/shutdown", 10)
        self.speed_multiplier_pub = self.create_publisher(Float32, "/kb/speed_multiplier", 10)
        self.extrusion_scale_pub = self.create_publisher(Float32, "/kb/extrusion_scale", 10)

        self.get_logger().info(
            "\n"
            "-----------------------------------\n"
            "Keyboard Node is running.\n"
            "Press 'l' then Enter to toggle detailed logging.\n"
            "Press 's <value>' then Enter to set print speed multiplier (e.g., 's 2.5').\n"
            "Press 'e <value>' then Enter to set extrusion scale factor (e.g., 'e 1.5').\n"
            "Press 'q' then Enter to quit.\n"
            "-----------------------------------"
        )

        self.main_loop()

    def main_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Enter command: ")
                if user_input == "l":
                    self.get_logger().info(
                        "User pressed 'l'. Sending LOG toggle signal to move_ur node."
                    )

                    log_msg = Bool()
                    log_msg.data = True
                    self.toggle_log_pub.publish(log_msg)

                elif user_input == "q":
                    self.get_logger().info(
                        "User pressed 'q'. Robot moving to HOME, program shutting down."
                    )

                    shutdown_msg = Bool()
                    shutdown_msg.data = True
                    self.shutdown_pub.publish(shutdown_msg)

                    sleep(0.5)
                    break

                elif user_input.startswith("s "):
                    match = re.match(r"s\s+([\d.]+)", user_input)
                    if match:
                        try:
                            speed_value = float(match.group(1))
                            if speed_value > 0:
                                self.get_logger().info(
                                    f"Setting print speed multiplier to {speed_value}"
                                )
                                
                                speed_msg = Float32()
                                speed_msg.data = speed_value
                                self.speed_multiplier_pub.publish(speed_msg)
                            else:
                                self.get_logger().warn("Speed multiplier must be greater than 0")
                        except ValueError:
                            self.get_logger().warn(f"Invalid speed value: '{match.group(1)}'")
                    else:
                        self.get_logger().warn("Invalid speed command format. Use 's <value>' (e.g., 's 1.5')")

                elif user_input.startswith("e "):
                    match = re.match(r"e\s+([\d.]+)", user_input)
                    if match:
                        try:
                            extrusion_value = float(match.group(1))
                            if extrusion_value >= 0:
                                self.get_logger().info(
                                    f"Setting extrusion scale factor to {extrusion_value}"
                                )
                                
                                extrusion_msg = Float32()
                                extrusion_msg.data = extrusion_value
                                self.extrusion_scale_pub.publish(extrusion_msg)
                            else:
                                self.get_logger().warn("Extrusion scale factor must be non-negative")
                        except ValueError:
                            self.get_logger().warn(f"Invalid extrusion value: '{match.group(1)}'")
                    else:
                        self.get_logger().warn("Invalid extrusion command format. Use 'e <value>' (e.g., 'e 0.8')")
                else:
                    self.get_logger().warn(f"Unknown command: '{user_input}'")

            except EOFError:
                break


def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()

    try:
        pass
    finally:
        keyboard_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
