
from g11_moveo import BCN3D_Moveo, Moveo_IK
import tkinter as tk
from tkinter import Scale, HORIZONTAL
import math
import time

# Assuming BCN3D_Moveo and Moveo_IK classes are defined elsewhere
arm = BCN3D_Moveo('/dev/ttyUSB0')
ik = Moveo_IK()

def on_slider_change(val):
    x = x_scale.get()
    y = y_scale.get()
    z = z_scale.get()
    
    angles = ik.point_to_angles(x, y, z)
    #ik.plot_manipulator_3d(*angles)
    arm.go_to(*angles)
    # You might need to adjust or remove the plotting method depending on your setup
    # ik.plot_manipulator_3d(*angles)


root = tk.Tk()
root.title("Robot Arm Control")

# Set a minimum size for the window
root.minsize(600, 400)

x_scale = Scale(root, from_=-0, to=1.0, resolution=0.01, orient=HORIZONTAL, label="X Position", command=on_slider_change)
#x_scale.set(0.3)  # Default value
x_scale.pack(fill='x')  # Ensure it fills the space along the x-axis

y_scale = Scale(root, from_=-0, to=1.0, resolution=0.01, orient=HORIZONTAL, label="Y Position", command=on_slider_change)
#y_scale.set(0.3)  # Default value
y_scale.pack(fill='x')  # Same for Y

z_scale = Scale(root, from_=-0.25, to=1.0, resolution=0.01, orient=HORIZONTAL, label="Z Position", command=on_slider_change)
#z_scale.set(0.1)  # Default value
z_scale.pack(fill='x')  # And Z

angles = ik.point_to_angles(0.25,0.25,0.25)
arm.go_to(*angles)
root.mainloop()