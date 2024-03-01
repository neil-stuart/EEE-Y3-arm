import serial
import time
import random

# Replace '/dev/ttyACM0' with the correct port for your system. 
# This might be something like 'COM3' on Windows.
ser = serial.Serial('/dev/ttyACM0', 115299, timeout=1)
time.sleep(2)  # Wait for the connection to initialize

def set_speed(motor_id, speed):
    command = f"S{motor_id},{speed};"
    ser.write(command.encode())

def set_direction(motor_id, direction):
    # direction: 0 for one direction, 1 for the opposite
    command = f"D{motor_id},{direction};"
    ser.write(command.encode())

# def set_enabled(motor_id, on_off):
#     # direction: 0 for one direction, 1 for the opposite
#     command = f"E{motor_id},{on_off};"
#     ser.write(command.encode())
#     while True:
#         if ser.in_waiting > 0:
#             response = ser.readline().decode().strip()
#             print(f"Motor {motor_id} Enabled: {response}")
#             break

def request_position(motor_id):
    command = f"P{motor_id};"
    ser.write(command.encode())
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Motor {motor_id} Position: {response}")
            break

def main():
    while True:
        motor_id = random.randint(1, 4)  # Select a random motor (1 to 4)
        speed = random.randint(0, 100)  # Select a random speed (0 to 100)
        direction = random.randint(0, 1)  # Select a random direction (0 or 1)
        #enabled = random.randint(0, 1)  # Select a random state
        print(f"Setting Motor {motor_id} - Speed: {speed}, Direction: {direction}")
        set_speed(motor_id, speed)  # Set the motor speed
        set_direction(motor_id, direction)  # Set the motor direction
        #set_enabled(motor_id, enabled)
        time.sleep(1)  # Wait a bit for the motor to respond
        print("\n ---------------\n")
        print(f"Requesting position for Motor {motor_id}")
        request_position(motor_id)  # Request position of the motor
        print("\n\n")
        time.sleep(2)  # Wait a bit before the next command

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program exited by user")
        ser.close()
