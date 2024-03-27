from g11_moveo import RS2_Ball_Tracking
import time

tracker = RS2_Ball_Tracking()
time.sleep(1)
while(True):
    try:
        print(tracker.get_xyz())
    except:
        tracker.stop()