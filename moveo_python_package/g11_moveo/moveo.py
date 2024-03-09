"""
Author: Neil Stuart
Group: 11
Email: b.stuart3@universityofgalway.ie
Date: March 2024

Project: Control Solution for Moveo Robotic Arm
University of Galway
"""

import serial
import math

class BCN3D_Moveo():

    STEPS_PER_REV = [ 2*200, 8*200, 1*(360/0.35), 200 ]  # TODO: ACCOUNT FOR MICROSTEPPING

    GEAR_RATIOS = [10, 5, 4, 2] # Gear ratio = input_angle/output_angle
    
    def __init__(self, port):
        self.nano = serial.Serial(port, 115200, timeout=1)

    def __set_direction(self, motor_id, direction):
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
                if len(parts) > 1 and parts[0] == 'P' + str(motor_id):
                    number_str = parts[1].replace('\n', '')
                    return int(number_str)

    def __move_n_steps(self, motor_id, n_steps, direction):
        self.__set_direction(motor_id,direction)
        command = f"N{motor_id},{n_steps};"
        self.nano.write(command.encode())

    def radians_to_steps(self, motor_id, radians):
        # Convert radians to steps
        steps_per_rad = self.STEPS_PER_REV[motor_id] * self.GEAR_RATIOS[motor_id] / (2 * math.pi)
        return int(radians * steps_per_rad)

    def go_to(self, m0, m1, m2, m3):
        desired_positions_radians = [m0, m1, m2, m3]
        desired_positions_steps = [self.radians_to_steps(i, rad) for i, rad in enumerate(desired_positions_radians)]
        
        print(desired_positions_steps)
        
        
        # Get current positions in steps
        current_positions = [self.request_position(i) for i in range(4)]

        # Calculate offsets in steps
        offsets = [current_position - desired for current_position, desired in zip(current_positions, desired_positions_steps)]

        directions = [1 if offset >= 0 else 0 for offset in offsets]
        
        for id, offset, direction in zip(range(4), offsets, directions):
            self.__move_n_steps(id, abs(offset), direction)

