import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2 

class CameraInterface(Node):

    def __init__(self):
        super().__init__('camera_interface')
        self.fps = 60
        self.timer = self.create_timer(1/self.fps, self.timer_callback)
        self.camera = cv2.VideoCapture(0) 
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.bridge = CvBridge()
        self.i = 1

    def timer_callback(self):
        ret, frame = self.camera.read()
        
        #publish
        if ret:
            img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            img.header.stamp = self.get_clock().now().to_msg()
            img.header.frame_id = 'Frame '+ str(self.i)      
            self.get_logger().info('Publishing: "%s"' % str(img.header.frame_id))
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(img)
            self.i += 1
        else:
            self.get_logger().info('No image')
        

def main(args=None):
    rclpy.init(args=args)

    camera_interface_node = CameraInterface()

    rclpy.spin(camera_interface_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()