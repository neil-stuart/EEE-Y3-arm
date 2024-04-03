from g11_moveo import TransformPoint, RS2_Ball_Tracking, Moveo_IK, BCN3D_Moveo
import time
import numpy as np
import matplotlib.pyplot as plt

WINDOW_SIZE = 15

tracker = RS2_Ball_Tracking(display=True)
arm = BCN3D_Moveo("/dev/ttyUSB0")
transform = TransformPoint()
ik = Moveo_IK()

class Plotter:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
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
        self.points_plot = self.ax.scatter(xs,ys,zs, color="red")

        # Show plot
        plt.pause(0.001)  # Pause to allow the plot to update
graph = Plotter()


if __name__ == "__main__":
    moving_avg_pts = []
    arm_target = np.array([0,0,0])

    time.sleep(3)
    
    moving_avg_pts = np.zeros((WINDOW_SIZE,3))

    try:
        while(True):

            ball_xyz = tracker.get_xyz()
            
            if ball_xyz:
                # Use a moving average to smooth out points.
                arm_point = transform.find_corresponding_point(ball_xyz)
                
                moving_avg_pts.insert(0,ball_xyz)
                
                moving_avg_pts = moving_avg_pts[:WINDOW_SIZE]
                
                graph.update(moving_avg_pts)

                mov_avg = np.mean(moving_avg_pts, axis=0)

                if(abs(np.linalg.norm(arm_point - mov_avg))>0.3):
                    continue # Do nothing if the point is an outlier - far from current average.
                else:
                    arm_target = mov_avg

                # Within Range: Check if the distance from the arms origin is less than the reaching distance
                if((np.array(arm_target)**2).sum()<0.5): 
                
                    angles = ik.point_to_angles(*arm_target)
                    
                    arm.go_to(*angles)
                    time.sleep(0.01)
                pass                
    except Exception as e:
        print(e)
        tracker.stop()