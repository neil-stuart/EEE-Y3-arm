import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import pyrealsense2 as rs

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'xyz_coordinates', 10)
        self.fps = 60  # Adjust according to your needs
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # Configure the streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.fps)
        # Start the pipeline
        self.pipeline.start(self.config)


        self.create_timer(1.0 / self.fps, self.publish_frame)

        self.add_on_set_parameters_callback(self.cleanup)
    def get_ball_xy(self, color_image):
        # Color threshold to find the ball
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_color = np.array([23, 100, 100])
        upper_color = np.array([47, 255, 255])
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            if radius > 10:  # Minimum size to consider
                return int(x), int(y)
        return None, None

    def publish_frame(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        # # Convert images to numpy arrays
        # depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Get the ball position in pixel coordinates
        x, y = self.get_ball_xy(color_image)

        if x is not None and y is not None:

            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            depth = depth_frame.get_distance(x, y)
            try:
                xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)

                # Publish XYZ coordinates
                msg = Float32MultiArray()
                msg.data = xyz
                self.get_logger().info(f"x:{xyz[0]:.2f},y:{xyz[1]:.2f},z:{xyz[2]:.2f}")
                self.publisher_.publish(msg)
            except:
                self.get_logger().info("Point not within depth FOV.")
        else:
            self.get_logger().info('Object not found')
            
    def cleanup(self):
        # Method to stop the RealSense pipeline
        self.get_logger().info('Shutting down: Stopping the RealSense pipeline.')
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessor()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
