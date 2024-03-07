import serial
import time
import random



# const int STEPPER_PARAMS[N_STEPPERS][3] = { 
#   // Stepper parameters given as MIN_POSITION, MAX_POSITION, MAX_FREQUENCY
#   {-1000,1000,1000},
#   {-680,680,500},
#   {-1500,1500,1500},
#   {-500,500,500} // Initialize params for here 
# };

class BCN3D_Moveo():
    def __init__(self, port, params=None):
        self.nano = serial.Serial(port, 115299, timeout=1)
        # motor,n_steps,max_speed,min_speed
        self.params = [[0,700,1000,300],[1,350,500,150],[2,1200,1500,450]]
        
        pass
        
    def get_ease_in_out_speed(self, x):
        if x > 0 and x < 1:
            return -4 * x * x + 4 *x
        return 0

    def set_frequency(self, motor_id, speed):
        command = f"F{motor_id},{speed};"
        self.nano.write(command.encode())
            
    # direction: 0 for one direction, 1 for the opposite
    def set_direction(self, motor_id, direction):
        command = f"D{motor_id},{direction};"
        self.nano.write(command.encode())


    def set_enabled(self, motor_id, on_off):
        command = f"E{motor_id},{on_off};"
        self.nano.write(command.encode())

    def request_position(self, motor_id):
        command = f"P{motor_id};"
        self.nano.write(command.encode())

        while True:
            if self.nano.in_waiting > 0:
                response = self.nano.readline().decode().strip()
                parts = response.split(',')
                if len(parts) > 1:
                    number_str = parts[1].replace('\n', '')
                    return int(number_str)
                    
    def move_n_steps(self, motor_id, n):
        command = f"N{motor_id},{n};"
        self.nano.write(command.encode())

    

def test_motors():
    arm = BCN3D_Moveo('/dev/ttyUSB0')
    time.sleep(2)
    motor = 1
    n_steps = 350
    direction = 1
    max_speed = 500
    min_speed = 50

    for motor,n_steps,max_speed,min_speed in arm.params:
        for direction in [0,1,1,0]:
            print(direction)
            start_pos = arm.request_position(motor)
            pos = start_pos
            arm.set_direction(motor,direction)
            end_pos = start_pos+(n_steps)*(1 if direction==0 else -1)

            time.sleep(0.01)

            arm.move_n_steps(motor,n_steps)

            while(pos!=end_pos):
                print(pos)
                pos = arm.request_position(motor)
                speed = arm.get_ease_in_out_speed(abs(pos-start_pos)/n_steps)*(max_speed-min_speed)+min_speed
                arm.set_frequency(motor,speed)

    arm.nano.close()


if __name__ == "__main__":
    try:
        test_motors()
    except KeyboardInterrupt:
        print("Program Finished.")
