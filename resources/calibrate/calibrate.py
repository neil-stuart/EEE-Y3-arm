import cv2
import numpy as np
import pyrealsense2 as rs
from g11_moveo import BCN3D_Moveo, Moveo_IK, RS2_Ball_Tracking
import time

arm_pos = [0,0,0]
tracked_pos = []
i = 0
tracker = RS2_Ball_Tracking(display=True)
time.sleep(3)

def save_point():
    global arm_pos
    global tracked_pos

    global i 

    xyz = tracker.get_xyz()
    if(xyz):
        tracked_pos = xyz
        x,z,y = xyz
        y = -y
        with open("./resources/calibrate/points.csv",'a') as f:
            f.write(f"\n{x},{y},{z},{arm_pos[0]},{arm_pos[1]},{arm_pos[2]}")
            print("Points appended to points.csv!")

        on_button_click()



    else:
        print('(Tracking) SAVE FAILED: Object not found!')


def run_arm_movement(x, y, z, arm):
    try:
        ik = Moveo_IK()
        angles = ik.point_to_angles(x, y, z)
        arm.go_to(*angles)
        print("Arm moved to position:", x, y, z)
    except Exception as e:
        print("Error moving arm: ", e)

def display_positions():
    print("Arm Position:", arm_pos)
    print("Tracked Position:", tracked_pos)

def on_button_click():
    global arm_pos
    global i
    global arm_points

    # Get the values from text fields
    x,y,z = arm_points[i]

    arm_pos = [x, y, z]

    i += 1

    run_arm_movement(x, y, z, arm)

    display_positions()

if __name__ == "__main__":

    arm = BCN3D_Moveo("/dev/ttyUSB0")
    
    arm_points = [[0.25,0,0.0],[0.25,0.25,0.0],[0.25,0,0.10],[0.25,0.25,0.10],[0.3,0.1,0.25],[0.3,-0.1,0.35],[0.1,0.3,0],[0.1,0.3,0.18],[0.1,0.3,0.2],[0.1,0.3,0.25]]
    
    # Create GUI
    import tkinter as tk

    root = tk.Tk()
    root.title("Arm Control")


    button_save_point = tk.Button(root, text="Save Point", command=lambda: save_point())
    button_save_point.grid(row=2, column=0, padx=10, pady=5)

    label_xyz = tk.Label(root, text="Enter XYZ:")
    label_xyz.grid(row=3, column=0, padx=10, pady=5)

    text_field_x = tk.Entry(root)
    text_field_x.grid(row=3, column=1, padx=10, pady=5)
    text_field_x.insert(tk.END, "0")

    text_field_y = tk.Entry(root)
    text_field_y.grid(row=3, column=2, padx=10, pady=5)
    text_field_y.insert(tk.END, "0")

    text_field_z = tk.Entry(root)
    text_field_z.grid(row=3, column=3, padx=10, pady=5)
    text_field_z.insert(tk.END, "0")

    button_move_arm = tk.Button(root, text="Move Arm", command=on_button_click)
    button_move_arm.grid(row=3, column=4, padx=10, pady=5)

    root.mainloop()
    tracker.stop()
