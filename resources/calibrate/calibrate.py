from g11_moveo import BCN3D_Moveo, Moveo_IK, RS2_Ball_Tracking
import time

import numpy as np
arm = BCN3D_Moveo("/dev/ttyUSB0")
arm_pos = [0,0,0]

tracked_pos = []

i = 0

tracker = RS2_Ball_Tracking(display=True)

ik = Moveo_IK()

time.sleep(3)

calibration_points = [ # Grid of points
    [0.3,0,0.0],
    [0.3,0.3,0],
    [0.3,0.3,0.3],

    [0.3,0.3,0.2],
]

def save_point():
    global tracked_pos
    global arm_pos
    global i
    xyz = tracker.get_xyz()
    
    

    if(xyz):
        xyz = np.round(xyz,3)
        tracked_pos = xyz
        x, y ,z = xyz
        print("Arm Position:", arm_pos)
        print("Tracked Position:", tracked_pos)

        with open("./resources/calibrate/points.csv",'a') as f:
            f.write(f"\n{x},{y},{z},{arm_pos[0]},{arm_pos[1]},{arm_pos[2]}")
            print("Calibration point appended to points.csv!")

        
        i += 1

        arm_pos= calibration_points[i]

        angles = ik.point_to_angles(*arm_pos)
        print(arm_pos)
        arm.go_to(*angles)
        
        

    else:
        print('(Tracking) SAVE FAILED: Object not found!')


if __name__ == "__main__":
    arm_pos= calibration_points[i]
    print(arm_pos)
    arm.go_to(*ik.point_to_angles(*calibration_points[i]))

    import tkinter as tk

    root = tk.Tk()
    root.title("Arm Control")

    button_save_point = tk.Button(root, text="Save & Next Point", command=lambda: save_point())
    button_save_point.grid(row=2, column=0, padx=10, pady=5)

    root.mainloop()
    tracker.stop()
