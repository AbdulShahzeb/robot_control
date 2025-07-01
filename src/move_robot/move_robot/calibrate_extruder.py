import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import math

# --- Constants ---
SHAFT_DIAMETER = 11.0
MICROSTEPPING = 16.0
STEPS_PER_REVOLUTION = 200.0 * MICROSTEPPING

# This is the value we are trying to calibrate!
# Start with the theoretical value.
STEPS_PER_MM = 106.0 #STEPS_PER_REVOLUTION / (math.pi * SHAFT_DIAMETER)

class ExtruderCalibrator(Node):

    def __init__(self):
        super().__init__('extruder_calibrator')
        # Publisher to the /stepper_speed topic 
        self.publisher_ = self.create_publisher(Float32, 'stepper/speed', 10)
        self.get_logger().info('Extruder Calibrator node started.')

    def extrude(self, distance_mm, speed_sps):
        """
        Commands the extruder to run at a specific speed for a calculated duration.
        
        Args:
            distance_mm (float): The target distance of filament to extrude.
            speed_sps (float): The speed to run the stepper in steps per second.
        """
        if speed_sps <= 0:
            self.get_logger().error("Speed must be positive.")
            return

        # 1. Calculate total steps needed
        total_steps = distance_mm * STEPS_PER_MM
        
        # 2. Calculate the required run time in seconds
        # Time = Total Distance (steps) / Speed (steps/sec)
        run_time_seconds = total_steps / speed_sps

        self.get_logger().info(f"Commanding extrusion of {distance_mm}mm.")
        self.get_logger().info(f"Calculated steps: {float(total_steps)}")
        self.get_logger().info(f"Running motor at {speed_sps} steps/sec for {run_time_seconds:.2f} seconds.")
        
        # Give user time to prepare
        self.get_logger().info("Starting in 5 seconds...")
        time.sleep(5)

        # 3. Create and publish the speed message to start the motor
        start_msg = Float32()
        start_msg.data = float(-1*speed_sps)
        self.publisher_.publish(start_msg)
        self.get_logger().info("Motor ON.")

        # 4. Wait for the calculated duration
        time.sleep(run_time_seconds)

        # 5. Publish a zero speed message to stop the motor
        stop_msg = Float32()
        stop_msg.data = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Motor OFF. Calibration extrude complete.")


def main(args=None):
    rclpy.init(args=args)
    calibrator_node = ExtruderCalibrator()

    # --- DEFINE CALIBRATION PARAMETERS HERE ---
    DISTANCE_TO_EXTRUDE = 50.0  # mm
    CALIBRATION_SPEED_SPS = 30 # A moderate speed in steps per second

    try:
        calibrator_node.extrude(DISTANCE_TO_EXTRUDE, CALIBRATION_SPEED_SPS)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motor as a failsafe if script is interrupted
        stop_msg = Float32()
        stop_msg.data = 0.0
        calibrator_node.publisher_.publish(stop_msg)
        
        calibrator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()