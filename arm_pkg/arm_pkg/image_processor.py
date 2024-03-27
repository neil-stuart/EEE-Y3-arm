import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from g11_moveo import RS2_Ball_Tracking

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.tracker = RS2_Ball_Tracking()

        self.create_timer(1.0 / self.fps, self.publish_location)

        self.add_on_set_parameters_callback(self.cleanup)

    def publish_location(self):
        xyz = self.tracker.get_xyz()
        if xyz:
            msg = Float32MultiArray()
            msg.data = xyz
            self.get_logger().info(f"x:{xyz[0]:.2f},y:{xyz[1]:.2f},z:{xyz[2]:.2f}")
            self.publisher_.publish(msg)
        else:
            self.get_logger().info(f"No position")
            
    def cleanup(self):
        # Method to stop the RealSense pipeline
        self.get_logger().info('Shutting down: Stopping the RealSense pipeline.')
        self.tracker.stop()

def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessor()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
