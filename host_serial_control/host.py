import serial
import time
import math
from g11_moveo import BCN3D_Moveo


def test_motors():
    pi = math.pi
    arm = BCN3D_Moveo('/dev/ttyUSB0')
    time.sleep(2)

    arm.go_to(-pi/2,  -pi/2, pi/2,  -pi/2)
    time.sleep(3)

    arm.go_to(0,0,0,0)
    time.sleep(3)

    # arm.go_to(-pi/2, pi/2, -pi/2, -pi/4)
    # time.sleep(3)

    # arm.go_to(0,0,0,0)

    print("Finished test.")
    arm.nano.close()


if __name__ == "__main__":
    try:
        test_motors()
    except KeyboardInterrupt:
        print("Program Finished.")
