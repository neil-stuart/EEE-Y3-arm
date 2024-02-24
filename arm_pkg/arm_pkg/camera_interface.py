import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from sensor_msgs.msg import CompressedImage
import cv2 

class CameraInterface(Node):

    def __init__(self):
        
        super().__init__('camera_interface')
        
        self.fps = 30 # 30 is max fps on this camera
        self.timer = self.create_timer(1/self.fps, self.timer_callback)

        self.camera = cv2.VideoCapture(0) 
        
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        
        self.bridge = CvBridge()
        
        self.i = 0

    def timer_callback(self):
        ret, frame = self.camera.read()

        if ret:
            
            img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')

            img.header.stamp = self.get_clock().now().to_msg()
            img.header.frame_id = str(self.i)      
            
            self.publisher_.publish(img)
        
            self.i += 1
            self.b = 0

        else:
            self.b += 1
            if self.b % 30:
                self.b = 0 
                self.get_logger().warn('No image for the last 30 attempts. ')
            pass
        

def main(args=None):
    rclpy.init(args=args)

    camera_interface_node = CameraInterface()

    rclpy.spin(camera_interface_node)

    # Destroy the node explicitly
    camera_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()