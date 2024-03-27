import cv2
import numpy as np
import pyrealsense2 as rs
from g11_moveo import BCN3D_Moveo, Moveo_IK, RS2_Ball_Tracking
import time

arm_pos = [0,0,0]
tracked_pos = []

tracker = RS2_Ball_Tracking()
time.sleep(3)

def save_point():
    global profile
    global arm_pos
    global tracked_pos

   
    xyz = tracker.get_xyz()
    if(xyz):
        tracked_pos = xyz
        x,y,z = xyz
        with open("./resources/calibrate/points.csv",'a') as f:
            f.write(f"\n{x},{y},{z},{arm_pos[0]},{arm_pos[1]},{arm_pos[2]}")
            print("Points appended to points.csv!")



    else:
        print('(Tracking) SAVE FAILED: Object not found!')

def get_ball_xy(color_image):
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    lower_color = np.array([23, 100, 100])
    upper_color = np.array([47, 255, 255])
    mask = cv2.inRange(hsv_image, lower_color, upper_color)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        if radius > 10:  
            return int(x), int(y)
    return None, None

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
    global tracked_pos

    # Get the values from text fields
    x = float(text_field_x.get())
    y = float(text_field_y.get())
    z = float(text_field_z.get())

    arm_pos = [x, y, z]
    tracked_pos = [0, 0, 0]  # Reset tracked position

    run_arm_movement(x, y, z, arm)

    display_positions()

if __name__ == "__main__":

    arm = BCN3D_Moveo("/dev/ttyUSB0")

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
