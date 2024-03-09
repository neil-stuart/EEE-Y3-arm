import serial
import time
import math
from g11_moveo import BCN3D_Moveo, Moveo_IK

def test_motors():
    pi = math.pi
    #arm = BCN3D_Moveo('/dev/ttyUSB0')
    ik = Moveo_IK()

    time.sleep(2)

    angles = ik.point_to_angles(0.3,0.3,0.10)

    #arm.go_to(*angles)

    ik.plot_manipulator_3d(*angles)
    
    time.sleep(3)

    #arm.go_to(0,0,0,0)
    time.sleep(3)

    # arm.go_to(-pi/2, pi/2, -pi/2, -pi/4)
    # time.sleep(3)

    # arm.go_to(0,0,0,0)

    print("Finished test.")


if __name__ == "__main__":
    try:
        test_motors()
    except KeyboardInterrupt:
        print("Program Finished.")
