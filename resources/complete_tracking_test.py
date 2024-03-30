from g11_moveo import TransformPoint, RS2_Ball_Tracking, Moveo_IK, BCN3D_Moveo
import time
import numpy as np


WINDOW_SIZE = 30

if __name__ == "__main__":
    arm = BCN3D_Moveo("/dev/ttyUSB0")
    transform = TransformPoint()
    tracker = RS2_Ball_Tracking()
    ik = Moveo_IK()
    time.sleep(3)
    
    moving_avg_pts = []

    try:
        while(True):

            ball_xyz = tracker.get_xyz()
            if ball_xyz:
                # Use a moving average to smooth out points.
                arm_point = transform.find_corresponding_point(ball_xyz)
                
                moving_avg_pts.insert(0,arm_point)
                moving_avg_pts = moving_avg_pts[:WINDOW_SIZE]
                
                mov_avg = np.mean(moving_avg_pts, axis=0)

                if(abs(np.linalg.norm(arm_point - mov_avg))>0.1):
                    pass # Do nothing if the point is an outlier - far from current average.
                else:
                    arm_target = mov_avg

                # Within Range: Check if the distance from the arms origin is less than the reaching distance
                if((np.array(arm_target)**2).sum()<0.6): 
                
                    angles = ik.point_to_angles(*arm_target)
                    
                    arm.go_to(*angles)
                    time.sleep(0.01)
                
    except:
        tracker.stop()