"""
Author: Neil Stuart
Group: 11
Email: b.stuart3@universityofgalway.ie
Date: March 2024

Project: Inverse Kinematics Solution for Moveo Robotic Arm
University of Galway
"""


# ROUGH WORK

# r = sqrt(a^2+b^2) [a, b in terms of angles of interest - Theta_1,2,3]
# Phi = 90-atan(b/a)
# Theta = Theta_0

# x = rsin(\Phi)cos(\Theta)
# y = rsin(\Phi)sin(\Theta)
# z = rcos(\Theta)

# Lets say x,y,z = [2,1,4]

# Then \Theta = atan(y/x) = atan(1/2)

# r = sqrt(4+1+16) = sqrt(21)

# SO, a^2+b^2 = 21

# Therefore (l_1*cos(\Theta_1)+l_2*cos(\Theta_2)+l_3*cos(\Theta_3))^2
#           +(l_1*sin(\Theta_1)+l_2*sin(\Theta_2)+l_3*sin(\Theta_3))^2
#           = 21
# THIS NEEDS TO BE SOLVED FOR \Theta_1, \Theta_2 and \Theta_3, i.e. a solution matrix needs to be made


# l1,l2,l3 = [1,1,1]

# x,y,z = [2,1,4]

import matplotlib.pyplot as plt
import numpy as np
import base64
from io import BytesIO

from scipy.optimize import minimize

class Moveo_IK():
    def __init__(self, lengths=None):
        """
        ### `__init__(self, lengths=None)`
        Initializes the Moveo_IK object.

        #### Arguments:
        - `lengths` (_list_ or _tuple_, optional): Lengths of the three manipulator links. Defaults to stored values.
        """
        if lengths:
            self.l1, self.l2, self.l3 = lengths
        else:
            self.l1 = 0.221 # Meters
            self.l2 = 0.223
            self.l3 = 0.12
        
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': '3d'})
        self.line1, = self.ax.plot([], [], [], 'ro-')
        self.line2, = self.ax.plot([], [], [], 'go-')
        self.line3, = self.ax.plot([], [], [], 'bo-')
        self.end_effector, = self.ax.plot([], [], [], 'ko')
        self.ax.set_title("Inverse Kinematics - Arm Coordinates")
        self.ax.set_xlim(-0.3, 0.3)
        self.ax.set_ylim(-0.3, 0.3)
        self.ax.set_zlim(-0.1, 0.4)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio for all axes

    def __objective(self, vars, x, y, a1, a2, a3):
        """
    ### `__objective(self, vars, x, y, a1, a2, a3)`
        Objective function for optimization.

    #### Arguments:
        - `vars` (list): Current angles of the three joints.
        - `x` (float): Target x-coordinate.
        - `y` (float): Target y-coordinate.
        - `a1` (float): Length of the first link.
        - `a2` (float): Length of the second link.
        - `a3` (float): Length of the third link.

    #### Returns:
        - float: The squared distance between the end effector's current position and the target position.

        """
        theta1, theta2, theta3 = vars
        x_end = a1 * np.cos(theta1) + a2 * np.cos(theta2+theta1) + a3 * np.cos(theta3+theta2+theta1)
        y_end = a1 * np.sin(theta1) + a2 * np.sin(theta2+theta1) + a3 * np.sin(theta3+theta2+theta1)
        return (x - x_end)**2 + (y - y_end)**2

    def __solve_3link_manipulator(self, a, b, l1, l2, l3):
        """
        Solves the inverse kinematics problem for a 3-link manipulator using optimization.
        
        Parameters:
        - a, b: The target coordinates (x, z) in the plane for the end effector to reach.
        - l1, l2, l3: The lengths of the three links of the manipulator.
        
        Returns:
        - The angles of the three joints (theta1, theta2, theta3) as a solution to the inverse kinematics problem.
        """
        initial_guess = [np.pi/2, 0, 0]
        result = minimize(self.__objective, initial_guess, args=(a, b, l1, l2, l3), bounds=[(0, np.pi),(-0.7*np.pi, 0.7*np.pi),(-0.7*np.pi, 0.7*np.pi)])
        theta1, theta2, theta3 = result.x
        return theta1-np.pi/2, theta2, theta3

    def update_plot_ax(self, theta0, theta1, theta2, theta3):
        """
        Plots a 3D representation of the manipulator based on the given joint angles.

        Parameters:
        - theta0: The base rotation angle.
        - theta1, theta2, theta3: The angles of the three joints.
        """
        theta1 = theta1 + np.pi / 2
        # Base coordinates
        x0, y0, z0 = 0, 0, 0

        x1 = self.l1 * np.cos(theta0) * np.cos(theta1)
        y1 = self.l1 * np.sin(theta0) * np.cos(theta1)
        z1 = self.l1 * np.sin(theta1)

        x2 = x1 + self.l2 * np.cos(theta0) * np.cos(theta1 + theta2)
        y2 = y1 + self.l2 * np.sin(theta0) * np.cos(theta1 + theta2)
        z2 = z1 + self.l2 * np.sin(theta1 + theta2)

        x3 = x2 + self.l3 * np.cos(theta0) * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + self.l3 * np.sin(theta0) * np.cos(theta1 + theta2 + theta3)
        z3 = z2 + self.l3 * np.sin(theta1 + theta2 + theta3)

        # Update the data of the existing lines
        self.line1.set_data([x0, x1], [y0, y1])
        self.line1.set_3d_properties([z0, z1])

        self.line2.set_data([x1, x2], [y1, y2])
        self.line2.set_3d_properties([z1, z2])

        self.line3.set_data([x2, x3], [y2, y3])
        self.line3.set_3d_properties([z2, z3])

        self.end_effector.set_data([x3], [y3])
        self.end_effector.set_3d_properties([z3])
        
        
    def get_plot_figure(self):
        return self.fig
    
    def point_to_angles(self, x, y, z):
        """Calculates the joint angles to reach a given point in 3D space.

        Args:
            x (float): Target x-coordinate.
            y (float): Target y-coordinate.
            z (float): Target z-coordinate.

        # Returns:
            tuple: Calculated angles for the base and the three joints.
        """
        theta0 = np.arctan2(y, x) # angle from positive x axis to the point
        a_target = np.sqrt(x**2 + y**2) # horizontal distance to the target
        b_target = z

        theta1, theta2, theta3 = self.__solve_3link_manipulator(a_target, b_target, self.l1, self.l2, self.l3)

        return theta0, theta1, theta2, theta3