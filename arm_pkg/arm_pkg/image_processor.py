import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from realsense2_camera_msgs.msg import RGBD 
import cv2
import time 
from cv_bridge import CvBridgeError
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            RGBD,
            'MOVEO/RS_CAM/aligned_depth_to_color/image_raw',
            self.image_callback,
            100)
        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'xyz', 10)
        self.frame_count = 0
        self.fps_start_time = time.time()
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.fps_start_time



        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            print("Error converting image")

        xys = get_ball_xys(cv_img)
        
        # Process image and publish xys location
        self.publisher_.publish(xys)

        if self.frame_count == 30:  
            fps = self.frame_count / elapsed_time

            self.get_logger().info('=================Recent State==================')
            self.get_logger().info(f'{fps:.1f} FPS')
            self.fps_start_time = current_time
            self.frame_count = 0
            self.get_logger().info('X Y S : ' + ' '.join(str(round(i,2)) for i in xys.data))

def get_ball_xys(cv_img):
    a = Float32MultiArray()
    img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    lower = np.array([0, 0, 0], np.uint8)
    upper = np.array([180, 255, 200], np.uint8)

    binary_img = cv2.inRange(img,lower,upper)
    contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # cv2.imshow('Binarized',binary_img)
    # cv2.waitKey(1)
    
    # Denoise

    a.data = track(cv2.cvtColor(img, cv2.COLOR_HSV2BGR),contours)
    return a

def track(cv_img, contours):
    for c in contours:
      area = cv2.contourArea(c)
      ((x,y), radius) = cv2.minEnclosingCircle(c)
      if (area > 5000):
        cv2.drawContours(cv_img, [c], -1, (255,0,255), 2)
        cx, cy = find_contour_center(c)
        cv2.circle(cv_img, (cx,cy), (int)(radius), (0,255,255), 3)
        cv2.circle(cv_img, (cx,cy), 5, (150,0,255), -1)

    cv2.imshow("Tracking", cv_img)
    cv2.waitKey(3)

    return [radius,cx,cy]

def find_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    return cx, cy

def main(args=None):
    rclpy.init(args=args)

    image_processor = ImageProcessor()

    rclpy.spin(image_processor)

    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()