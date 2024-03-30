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

class BCN3D_Moveo:
    """
    This class provides an interface to control the BCN3D Moveo robotic arm via serial communication.
    v1 = stepper_ctrlv1.ino, v2 = stepper_ctrlv2.ino
    """

    STEPS_PER_REV = [2*200, 8*200, 1*(360/0.35), 2*200]  # Steps per revolution for each motor. TODO: Account for microstepping.
    GEAR_RATIOS = [10, 5, 4, 2]  # Gear ratio for each motor (input_angle/output_angle).
    
    def __init__(self, port, v2=True):
        """
        Initializes the BCN3D_Moveo object by setting up the serial connection.
        
        Args:
            port (str): The serial port to connect to.
        """
        self.v2 = v2
        self.nano = serial.Serial(port, 115200, timeout=1)

    def __set_direction(self, motor_id, direction):
        """
        Sets the direction of rotation for a specified motor.
        
        Args:
            motor_id (int): The ID of the motor (0-3).
            direction (int): The direction to set (0 or 1).
        """
        command = f"D{motor_id},{direction};"
        self.nano.write(command.encode())

    def set_enabled(self, motor_id, on_off):
        """
        Enables or disables a specified motor.
        
        Args:
            motor_id (int): The ID of the motor (0-3).
            on_off (int): Enable (1) or disable (0) the motor.
        """
        command = f"E{motor_id},{on_off};"

        self.nano.write(command.encode())

    def request_position(self, motor_id):
        """
        Requests the current position of a specified motor.
        
        Args:
            motor_id (int): The ID of the motor (0-3).
        
        Returns:
            int: The current position of the motor in steps.
        """
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
        """
        Moves a specified motor a number of steps in a given direction.
        
        Args:
            motor_id (int): The ID of the motor (0-3).
            n_steps (int): The number of steps to move.
            direction (int): The direction of movement (0 or 1).
        """
        self.__set_direction(motor_id, direction)
        command = f"N{motor_id},{n_steps};"
        self.nano.write(command.encode())

    def __change_setpoint(self,motor_id,setpoint):
        command = f"S{motor_id},{setpoint};"
        self.nano.write(command.encode())

    def __radians_to_steps(self, motor_id, radians):
        """
        Converts radians to steps for a given motor.
        
        Args:
            motor_id (int): The ID of the motor (0-3).
            radians (float): The angle in radians.
        
        Returns:
            int: The equivalent number of steps for the given angle.
        """
        steps_per_rad = self.STEPS_PER_REV[motor_id] * self.GEAR_RATIOS[motor_id] / (2 * math.pi)
        return int(radians * steps_per_rad)

    def go_to(self, m0, m1, m2, m3):
        """
        Moves the robotic arm to the specified angles for each motor.
        
        Args:
            m0, m1, m2, m3 (float): The desired angles in radians for motors 0 to 3, respectively.
        """
        # Convert desired positions from radians to steps
        desired_positions_radians = [m0, m1, m2, m3]
        desired_positions_steps = [self.__radians_to_steps(i, rad) for i, rad in enumerate(desired_positions_radians)]
        
        if self.v2:
            for id,setpoint in zip(range(4),desired_positions_steps):
                self.__change_setpoint(id,setpoint)
        else:
            # Get current positions in steps
            current_positions = [self.request_position(i) for i in range(4)]

            # Calculate the difference in steps
            offsets = [current_position - desired for current_position, desired in zip(current_positions, desired_positions_steps)]

            # Determine direction for each motor
            directions = [1 if offset >= 0 else 0 for offset in offsets]
            
            # Move each motor by the calculated offset in the determined direction
            for id, offset, direction in zip(range(4), offsets, directions):
                self.__move_n_steps(id, abs(offset), direction)


