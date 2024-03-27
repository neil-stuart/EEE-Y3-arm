import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import sys

class RS2_Ball_Tracking():
    def __init__(self, lower_hsv=[23, 100, 100], upper_hsv=[47, 255, 255], fps=60, width=848, height=480):
    
        # Initialize the RealSense pipeline
        self.lower_hsv = np.array(lower_hsv)
        self.upper_hsv = np.array(upper_hsv)
        self.fps = fps
        self.width = width
        self.height = height

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color,self.width, self.height, rs.format.bgr8,self.fps)
        
        self.depth_min = 0.12 #meter
        self.depth_max = 2.5 #meter
        
        # Start streaming
        self.pipeline.start(self.config)
        self.profile = self.pipeline.get_active_profile()
        self.running = True
        self.xyz = None
        self.tracking_thread = threading.Thread(target=self.__get_ball_xyz)
        self.tracking_thread.start()

    def get_xyz(self):
        if self.xyz:
            return self.xyz
        
        return None
    
    def stop(self):
        self.pipeline.stop()
        self.running = False

    def __get_ball_xyz(self):
        while self.running:
            frames = self.pipeline.wait_for_frames() 
            
            depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
            
            depth_intrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            color_intrin = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

            depth_to_color_extrin =  self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( self.profile.get_stream(rs.stream.color) )
            color_to_depth_extrin =  self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( self.profile.get_stream(rs.stream.depth) )

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            color_image = np.asanyarray(color_frame.get_data())

            # Colour threshold the ball and return x, y pixel coordinates
            xy = self.__get_xy(color_image)
            if xy:
                x, y = xy
                # Convert from color to depth pixel coordinate
                x,y = rs.rs2_project_color_pixel_to_depth_pixel(
                    depth_frame.get_data(), depth_scale,
                    self.depth_min, self.depth_max,
                    depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [x-30,y+20])

                # TODO: Subsampling before projection to smooth data, and increase performance

                # Project depth pixel point to the 3D space
                try:
                    xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, [int(x),int(y)], depth_frame.get_distance(int(x),int(y)))
                    self.xyz = xyz
                except:
                    self.xyz = None

        self.pipeline.stop()
        

    def __get_xy(self, color_image):
        # Color threshold to find the ball
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)
    
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            if radius > 10:  # Minimum size to consider
                return int(x), int(y)
                
        return None
