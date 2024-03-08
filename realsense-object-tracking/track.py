import cv2
import numpy as np
import pyrealsense2 as rs
import datetime as t
import matplotlib.pyplot as plt
import threading
import time

pts = [
    [0,0,0]
      ]

class Plotter:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X (m)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")

        # Set limits for the plot
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(0, 1.5)
        self.ax.set_zlim(-1, 1)

        self.points_plot = None

    def update(self, points):

        if self.points_plot is not None:
            self.points_plot.remove()  # Remove the previous plot

        # Set axis labels

        # Original zip
        xs, ys, zs = zip(*points[-30:])


        # Plot the points
        self.points_plot = self.ax.scatter(xs,zs,[-y for y in ys], color="red")

        # Show plot
        plt.pause(0.001)  # Pause to allow the plot to update

def get_ball_xy(cv_img):
    img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    lower = np.array([23, 100, 87], np.uint8)
    upper = np.array([47, 255, 255], np.uint8)

    binary_img = cv2.inRange(img,lower,upper)
    contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    x,y = track(cv2.cvtColor(img, cv2.COLOR_HSV2BGR),contours)

    return [x,y]

def track(cv_img, contours):
    prev_biggest_area = 0
    biggest_contour = None
    global prev_pos
    cx, cy = [0,0]
    for c in contours:
      area = cv2.contourArea(c)
      if (area > prev_biggest_area):
        prev_biggest_area = area
        biggest_contour = c

    if prev_biggest_area<50:
        cv2.circle(cv_img, prev_pos, 5, (150,0,255), -1)
        cv2.imshow("Tracking", cv_img)
        return prev_pos

    try:
        ((x,y), radius) = cv2.minEnclosingCircle(biggest_contour)
        cv2.drawContours(cv_img, [biggest_contour], -1, (255,0,255), 2)
        cx, cy = find_contour_center(biggest_contour)
        cv2.circle(cv_img, (cx,cy), (int)(radius), (0,255,255), 3)
        cv2.circle(cv_img, (cx,cy), 5, (150,0,255), -1)

        cv2.imshow("Tracking", cv_img)
        prev_pos = (cx,cy)
        return cx,cy
    
    finally:
        return prev_pos

def find_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    return cx, cy

def main(pipeline, profile):

    global pts

    # There values are needed to calculate the mapping
    
    depth_min = 0.12 #meter
    depth_max = 2.5 #meter
    
    try:
        while True:
            frames = pipeline.wait_for_frames() 
            
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            
            depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

            depth_to_color_extrin =  profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.color) )
            color_to_depth_extrin =  profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.depth) )

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Colour threshold the ball and return x, y pixel coordinates
            x, y = get_ball_xy(color_image)

            # Convert from color to depth pixel coordinate
            x,y = rs.rs2_project_color_pixel_to_depth_pixel(
                depth_frame.get_data(), depth_scale,
                depth_min, depth_max,
                depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [x-30,y+20])

            # TODO: Subsampling before projection to smooth data, and increase performance

            # Project depth pixel point to the 3D space
            try:
                color_point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [int(x),int(y)], depth_frame.get_distance(int(x),int(y)))
                pts.append(color_point_3d)
                pts = pts[-50:]
                            #[print(round(color_point_3d[i],2),","," ") for i in range(3)]
                #print("\n")

                # Subsampling
                # depth_image = cv2.resize(depth_image, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
                
                depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=0.1),cv2.COLORMAP_AUTUMN)

                cv2.circle(depth_image, (int(x),int(y)), 5, (255,100,0), -1)

                # Show image
                cv2.imshow('Depth Map', depth_image)

                # Close window when 'q' is pressed
                
            except:
                print("Point not within depth FOV.")
                continue

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop streaming
        write_csv_points(pts)
        pipeline.stop()
        cv2.destroyAllWindows()

def write_csv_points(pts):
    # Convert each element of point to a string
    points_str = [[str(element) for element in point] for point in pts]
    
    with open(filename, 'a') as f:
        [f.write(",".join(point)+"\n") for point in points_str]

def graphing_func():
    
    global pts    
    graph = Plotter()
    while True:
        time.sleep(0.001)
        graph.update(pts)

if __name__ == "__main__":
    prev_pos = (0,0)

    filename = t.datetime.now().strftime("./%H:%M:%S") + ".csv"
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)

    # Start streaming
    pipeline.start(config)
    profile = pipeline.get_active_profile()
    
    # Example usage
    th = threading.Thread(target=graphing_func)

    th.start()
    main(pipeline, profile)