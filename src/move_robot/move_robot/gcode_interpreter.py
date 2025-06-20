#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import math
import os


class GCodeInterpreter(Node):
    def __init__(self):
        super().__init__("gcode_interpreter")

        # Publishers
        self.pose_publisher = self.create_publisher(Twist, "ur10e/pose", 10)
        self.stepper_publisher = self.create_publisher(Float32, "/stepper/speed", 10)

        # Subscriber for robot movement status
        self.movement_subscriber = self.create_subscription(
            Bool, "ur10e/is_moving", self.robot_moving_callback, 10
        )

        # Parameters
        self.declare_parameter("gcode_file", "print.gcode")
        self.declare_parameter("x_offset", 150.0)
        self.declare_parameter("y_offset", -50.0)
        self.declare_parameter("z_offset", 200.0)

        # Constants for stepper motor calculation
        self.SHAFT_DIAMETER = 11.0
        self.MICROSTEPPING = 16.0
        self.STEPS_PER_REVOLUTION = 200.0 * self.MICROSTEPPING
        self.STEPS_PER_MM = self.STEPS_PER_REVOLUTION / (math.pi * self.SHAFT_DIAMETER)

        # Offsets from robot base to print bed origin (in mm)
        self.X_OFFSET = (
            self.get_parameter("x_offset").get_parameter_value().double_value
        )
        self.Y_OFFSET = (
            self.get_parameter("y_offset").get_parameter_value().double_value
        )
        self.Z_OFFSET = (
            self.get_parameter("z_offset").get_parameter_value().double_value
        )

        # State variables
        self.current_position = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.current_feedrate = 0.0  # mm/min
        self.current_extrusion = 0.0  # mm
        self.prev_is_moving = False
        self.is_executing = False

        # G-code command queue and execution state
        self.gcode_commands = []
        self.command_index = 0

        # Load and parse G-code file
        self.load_gcode_file()

        self.get_logger().info("G-code Interpreter Node initialized")

    def load_gcode_file(self):
        """Load and parse the G-code file"""
        gcode_file = self.get_parameter("gcode_file").get_parameter_value().string_value

        if not os.path.exists(gcode_file):
            self.get_logger().error(f"G-code file not found: {gcode_file}")
            return

        try:
            with open(gcode_file, "r") as f:
                for line_num, line in enumerate(f, 1):
                    parsed = self.parse_gcode_line(line)
                    if parsed:
                        parsed["line_number"] = line_num
                        self.gcode_commands.append(parsed)

            self.get_logger().info(
                f"Loaded {len(self.gcode_commands)} G-code commands from {gcode_file}"
            )

        except Exception as e:
            self.get_logger().error(f"Error loading G-code file: {str(e)}")

    def parse_gcode_line(self, line):
        """Parse a single line of G-code to extract command parameters"""
        # Remove comments and whitespace
        line = line.split(";")[0].strip()
        if not line:
            return None

        parts = line.split()
        command = {}

        for part in parts:
            if part:
                letter = part[0].upper()
                try:
                    value = float(part[1:])
                    command[letter] = value
                except (ValueError, IndexError):
                    # Handle cases with no value, like G90
                    command[letter] = None

        return command if command else None

    def robot_moving_callback(self, msg):
        """Callback for robot movement status updates"""
        curr_is_moving = msg.data

        # Detect transition from moving to stopped
        if self.prev_is_moving and not curr_is_moving:
            self.get_logger().info(
                "Robot has stopped moving. Proceeding with next action."
            )
            if self.is_executing:
                self.execute_next_command()

        self.prev_is_moving = curr_is_moving

    def start_execution(self):
        """Start executing the G-code commands"""
        if not self.gcode_commands:
            self.get_logger().warn("No G-code commands to execute")
            return

        if self.is_executing:
            self.get_logger().warn("Execution already in progress")
            return

        self.get_logger().info("Starting G-code execution")
        self.is_executing = True
        self.command_index = 0

        # Execute the first command
        self.execute_next_command()

    def stop_execution(self):
        """Stop G-code execution"""
        speed_msg = Float32()
        speed_msg.data = 0.0
        self.stepper_publisher.publish(speed_msg)

    def execute_next_command(self):
        """Execute the next G-code command in the queue"""
        if not self.is_executing or self.command_index >= len(self.gcode_commands):
            if self.command_index >= len(self.gcode_commands):
                self.get_logger().info("G-code execution completed!")
                self.is_executing = False
            return

        command = self.gcode_commands[self.command_index]
        self.get_logger().info(
            f"Executing command {self.command_index + 1}/{len(self.gcode_commands)}: {command}"
        )

        # Process the command
        self.process_command(command)

        # Move to next command
        self.command_index += 1

    def process_command(self, command):
        """Process a single G-code command"""
        # Store previous state
        prev_position = self.current_position.copy()
        prev_extrusion = self.current_extrusion

        # Update feedrate if present
        if "F" in command and command["F"] is not None:
            self.current_feedrate = command["F"]
            self.get_logger().debug(
                f"Updated feedrate to {self.current_feedrate} mm/min"
            )

        # Update position
        if "X" in command and command["X"] is not None:
            self.current_position["X"] = command["X"]
        if "Y" in command and command["Y"] is not None:
            self.current_position["Y"] = command["Y"]
        if "Z" in command and command["Z"] is not None:
            self.current_position["Z"] = command["Z"]

        # Update extrusion if present
        if "E" in command and command["E"] is not None:
            self.current_extrusion = command["E"]

        # Handle different G-code commands
        if "G" in command and command["G"] in [0, 1]:
            self.execute_movement(prev_position, prev_extrusion)

    def execute_movement(self, prev_position, prev_extrusion):
        """Execute a movement command (G0 or G1)"""
        # Calculate movement distance
        delta_x = self.current_position["X"] - prev_position["X"]
        delta_y = self.current_position["Y"] - prev_position["Y"]
        delta_z = self.current_position["Z"] - prev_position["Z"]
        xyz_move_distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        # Calculate extrusion
        delta_e = self.current_extrusion - prev_extrusion

        # Calculate stepper speed
        stepper_speed = 0.0

        if xyz_move_distance > 0 and delta_e > 0 and self.current_feedrate > 0:
            # Calculate move time
            move_time_minutes = xyz_move_distance / self.current_feedrate
            move_time_seconds = move_time_minutes * 60

            # Calculate extrusion speed
            extrusion_speed_mmps = delta_e / move_time_seconds

            # Convert to steps per second
            stepper_speed = extrusion_speed_mmps * self.STEPS_PER_MM

        # Publish stepper speed
        speed_msg = Float32()
        speed_msg.data = float(stepper_speed)
        self.stepper_publisher.publish(speed_msg)

        # Publish robot pose (convert to robot coordinates)
        pose_msg = Twist()
        pose_msg.linear.x = self.current_position["X"] + self.X_OFFSET
        pose_msg.linear.y = self.current_position["Y"] + self.Y_OFFSET
        pose_msg.linear.z = self.current_position["Z"] + self.Z_OFFSET

        self.pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = GCodeInterpreter()
        node.start_execution()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_execution()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
