from g11_moveo import BCN3D_Moveo, Moveo_IK
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import *

"""
Author: Neil Stuart
Group: 11
Email: n.stuart3@universityofgalway.ie
Date: March 2024

Project: Control Solution for Moveo Robotic Arm
University of Galway
"""

COM_PORT = '/dev/ttyUSB0'

arm = BCN3D_Moveo(COM_PORT)
ik = Moveo_IK()
arm_pos = [0,0,0]
points = [[0,0,0]]

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
        
    def update_arm(self, event):
        global arm_pos
        arm_pos[0] = self.x_slider.get()
        arm_pos[1] = self.y_slider.get()
        arm_pos[2] = self.z_slider.get()
        angles = ik.point_to_angles(*arm_pos)
        self.update_func(*angles) 
        self.arm_move_function(*angles)
        ik_canvas.draw()



if __name__ == "__main__":
    root_widget = Tk()
    root_widget.title("G11: Robotic Arm GUI")

    # Set window color
    root_widget.configure(bg='white')

    ik.update_plot_ax(0,0,0,0)
    ik_fig = ik.get_plot_figure()

    # Create a container frame for the three widgets
    container_frame = Frame(root_widget, padx=50, pady=50)
    container_frame.grid(row=3, column=0, columnspan=2)
    container_frame.configure(bg='white')

    # Place the arm_control_widget in the container frame
    arm_control_widget = Arm_Control(lambda theta0, theta1, theta2, theta3: arm.go_to(theta0,theta1,theta2,theta3), container_frame, update_func=ik.update_plot_ax)
    arm_control_widget.grid(row=0, column=0)
    arm_control_widget.configure(bg='white')

    # Place ik_canvas in the container frame
    ik_canvas = FigureCanvasTkAgg(figure=ik_fig, master=container_frame)
    ik_canvas.get_tk_widget().grid(row=0, column=1)
    
    root_widget.mainloop()