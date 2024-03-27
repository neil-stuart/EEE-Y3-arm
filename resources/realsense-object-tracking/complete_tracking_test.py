from g11_moveo import TransformPoint, RS2_Ball_Tracking, Moveo_IK, BCN3D_Moveo
import time
import numpy as np
if __name__ == "__main__":
    arm = BCN3D_Moveo("/dev/ttyUSB0")
    transform = TransformPoint()
    tracker = RS2_Ball_Tracking()
    ik = Moveo_IK()
    time.sleep(3)
    try:
        while(True):

            ball_xyz = tracker.get_xyz()
            if ball_xyz:
                arm_target = transform.find_corresponding_point(ball_xyz)

                if((np.array(arm_target)**2).sum()<0.55):
                
                    angles = ik.point_to_angles(*arm_target)
                    
                    arm.go_to(*angles)
                
                    time.sleep(3)
    except:
        tracker.stop()