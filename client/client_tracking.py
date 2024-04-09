from g11_moveo import BCN3D_Moveo, RS2_Ball_Tracking, TransformPoint, Moveo_IK

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from tkinter import *
from PIL import Image, ImageTk

import time
import threading
import cv2
import csv

"""
Author: Neil Stuart
Group: 11
Email: n.stuart3@universityofgalway.ie
Date: March 2024

Project: Control Solution for Moveo Robotic Arm
University of Galway
"""

arm = BCN3D_Moveo('/dev/ttyUSB0')
tracker = RS2_Ball_Tracking()
ik = Moveo_IK()
arm_pos = [0,0,0]
points = [[0,0,0]]

tracking = False

def load_calibration_points(file_path):
    points_A = []
    points_B = []

    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            # Extract tracked and arm positions from each row
            tracked_pos = [float(row['trackedX']), float(row['trackedY']), float(row['trackedZ'])]
            arm_pos = [float(row['armX']), float(row['armY']), float(row['armZ'])]

            # Append to respective lists
            points_A.append(tracked_pos)
            points_B.append(arm_pos)

    return points_A, points_B

A, B = load_calibration_points("/calibration/points.csv")

transform = TransformPoint(A, B)

class VideoStreamWidget(Frame):
    def __init__(self, parent, image_func, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.current_image = None  # current image from the camera
        self.panel = Label(self)  # initialize image panel
        self.panel.pack(padx=10, pady=10)
        self.image_func = image_func

    def video_loop(self):
        """ Get frame from the video stream and show it in Tkinter """

        frame = self.image_func()  # read frame from video stream

        if frame is not None:  # frame captured without any errors
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)  # convert colors from BGR to RGBA
            self.current_image = Image.fromarray(cv2image)  # convert image for PIL
            imgtk = ImageTk.PhotoImage(image=self.current_image, master=self)  # convert image for tkinter
            self.panel.imgtk = imgtk  # anchor imgtk so it does not be deleted by garbage-collector
            self.panel.config(image=imgtk)  # show the image
            
        self.after(30, self.video_loop)  # call the same function after 30 milliseconds

    def start(self):
        """Start the video stream"""
        self.video_loop()



class Arm_Control(Frame):

    def __init__(self, arm_move_function, parent, update_func=None, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.arm_move_function = arm_move_function
        self.update_func = update_func
        self.x_slider = Scale(self, from_=-0.4,resolution=0.005,sliderlength=50, length=300,to=0.4, orient="horizontal", label="X (m)")
        self.y_slider = Scale(self, from_=-0.4,resolution=0.005,sliderlength=50,length=300, to=0.4, orient="horizontal", label="Y (m)")
        self.z_slider = Scale(self, from_=0.55, resolution=0.005,sliderlength=50,length=300,to=-0.25, orient="vertical", label="Z (m)")
        self.z_slider.set(0.3)
        self.x_slider.grid(row=0, column=0, padx=10, pady=15)
        self.y_slider.grid(row=1, column=0, padx=10, pady=15)
        self.z_slider.grid(row=0, column=1, rowspan=2, padx=10, pady=15)
        
        self.x_slider.bind("<Motion>", self.update_arm)
        self.y_slider.bind("<Motion>", self.update_arm)
        self.z_slider.bind("<Motion>", self.update_arm)
        self.x_slider.configure(bg='white')
        self.y_slider.configure(bg='white')
        self.z_slider.configure(bg='white')
        
        self.tracking = True
        self.update_sliders()

    def update_arm(self, event):
        arm_pos[0] = self.x_slider.get()
        arm_pos[1] = self.y_slider.get()
        arm_pos[2] = self.z_slider.get()
        angles = ik.point_to_angles(*arm_pos)
        self.update_func(*angles) 
        self.arm_move_function(*angles)

    def update_sliders(self):
        global tracking
        if tracking:
            self.x_slider.set(arm_pos[0])
            print(arm_pos[0])
            self.y_slider.set(arm_pos[1])
            self.z_slider.set(arm_pos[2])
        self.after(100, self.update_sliders)

class Scatter3DPlot:
    def __init__(self):
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.scatter = None
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # Set limits for the plot
        self.ax.set_xlim(-0.75, 0.75)
        self.ax.set_ylim(0, 1.5)
        self.ax.set_zlim(-0.75, 0.75)
        self.ax.set_title("Tracked Ball Location - Camera Coordinates")

    def get_figure(self):
        return self.fig

    def update_plot(self, x, y, z):
        if self.scatter is None:
            self.scatter = self.ax.scatter(x, y, z)
        else:
            self.scatter._offsets3d = (x, y, z)
        
def track_thread(update_func):
    global points
    global tracking
    while True:
        new_point = tracker.get_xyz()
        if new_point:
            transformed_pts = [transform.find_corresponding_point(new_point)]
            new_point = [new_point[0],new_point[2],-new_point[1]]
            points.insert(0,new_point)

            points = points[:30]
            
            xs, ys, zs = zip(*points)

            update_func(xs,ys,zs)
            
            ik.update_pt(*zip(*transformed_pts))
            
            
            points_canvas.draw()

            if tracking:
                # Within Range: Check if the distance from the arms origin is less than the reaching distance
                angles = ik.point_to_angles(*transformed_pts[0])
                ik.update_plot_ax(*angles)
                    
                if((transformed_pts[0])**2).sum()<0.5: 
                    # Get average of points, move to that location,
                    global arm_pos
                    arm_pos = transformed_pts[0]
                    arm.go_to(*angles)
                
            ik_canvas.draw()
        time.sleep(0.01)

def toggle():
    global tracking
    if tracking_toggle_btn.config('relief')[-1] == 'sunken':
        tracking = False
        tracking_toggle_btn.config(relief="raised")
    else:
        tracking = True
        tracking_toggle_btn.config(relief="sunken")

if __name__ == "__main__":
    
    root_widget = Tk()
    root_widget.title("Robotic Arm GUI")
    # Set window color
    root_widget.configure(bg='white')
    toolbar_widget = Frame(master=root_widget)
    toolbar_widget.grid(row=0,columnspan=2,pady=10,padx=10)

    tracking_toggle_btn = Button(master=toolbar_widget,text="Toggle Tracking", width=12, relief="raised", command=toggle)
    tracking_toggle_btn.grid(row=0,column=0)

    color_image_widget = VideoStreamWidget(root_widget, tracker.get_last_color_frame)
    depth_image_widget = VideoStreamWidget(root_widget, tracker.get_last_depth_frame)
    
    color_label = Label(root_widget, text="Color Stream")
    depth_label = Label(root_widget, text="Depth Stream")
    
    color_label.grid(row=1, column=0, pady=20)
    color_label.configure(bg='white')
    depth_label.grid(row=1, column=1, pady=20)
    depth_label.configure(bg='white')
    
    color_image_widget.grid(row=2, column=0)
    depth_image_widget.grid(row=2, column=1)
    
    color_image_widget.start()
    depth_image_widget.start()

    ik_fig = ik.get_plot_figure()

    points_plot = Scatter3DPlot()
    points_fig = points_plot.get_figure()
    
    # Create a container frame for the three widgets
    container_frame = Frame(root_widget, padx=50, pady=50)
    container_frame.grid(row=3, column=0, columnspan=2)
    container_frame.configure(bg='white')

    # Place the arm_control_widget in the container frame
    #arm.go_to(theta0, theta1, theta2, theta3)
    arm_control_widget = Arm_Control(lambda theta0, theta1, theta2, theta3: arm.go_to(theta0,theta1,theta2,theta3), container_frame, update_func=ik.update_plot_ax)
    arm_control_widget.grid(row=0, column=0)
    arm_control_widget.configure(bg='white')

    # Place ik_canvas in the container frame
    ik_canvas = FigureCanvasTkAgg(figure=ik_fig, master=container_frame)
    ik_canvas.get_tk_widget().grid(row=0, column=1)

    # Place points_canvas in the container frame
    points_canvas = FigureCanvasTkAgg(figure=points_fig, master=container_frame)
    points_canvas.get_tk_widget().grid(row=0, column=2)
    
    # Thread to continuously update the plot of tracked point.
    t = threading.Thread(target=track_thread,args=[points_plot.update_plot])
    t.start()

    root_widget.mainloop()