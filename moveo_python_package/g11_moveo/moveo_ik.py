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
from scipy.optimize import minimize

class Moveo_IK():
    def __init__(self, lengths=None):
        if lengths:
            self.l1, self.l2, self.l3 = lengths
        else:
            self.l1 = 4 # Meters
            self.l2 = 3
            self.l3 = 5

    def __objective(self, vars, x, y, a1, a2, a3):
        theta1, theta2, theta3 = vars
        x_end = a1 * np.cos(theta1) + a2 * np.cos(theta2+theta1) + a3 * np.cos(theta3+theta2+theta1)
        y_end = a1 * np.sin(theta1) + a2 * np.sin(theta2+theta1) + a3 * np.sin(theta3+theta2+theta1)
        return (x - x_end)**2 + (y - y_end)**2

    def __solve_3link_manipulator(self, a, b, l1, l2, l3):
        initial_guess = [np.pi/2, 0, 0]
        result = minimize(self.__objective, initial_guess, args=(a, b, l1, l2, l3), bounds=[(0, np.pi),(-np.pi/2, np.pi/2),(-np.pi/2, np.pi/2)])
        theta1, theta2, theta3 = result.x
        return theta1, theta2, theta3

    # This was used for testing, and is not really useful anymore
    def _plot_manipulator_2d(self, theta1, theta2, theta3):
        # Calculate joint positions
        #theta1, theta2, theta3 =[np.pi/2, np.pi/2, np.pi/2]
        x0, y0 = 0, 0 # Base
        x1, y1 = self.l1 * np.cos(theta1), self.l1 * np.sin(theta1)
        x2, y2 = x1 + self.l2 * np.cos(theta2+theta1), y1 + self.l2 * np.sin(theta2+theta1)
        x3, y3 = x2 + self.l3 * np.cos(theta3+theta2+theta1), y2 + self.l3 * np.sin(theta3+theta2+theta1)
        
        # Plotting
        plt.plot([x0, x1], [y0, y1], 'ro-') # Link 1
        plt.plot([x1, x2], [y1, y2], 'go-') # Link 2
        plt.plot([x2, x3], [y2, y3], 'bo-') # Link 3
        plt.plot(x3, y3, 'ko') # End-effector
        
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.xlabel('A')
        plt.ylabel('B')
        plt.title('3-Link Manipulator')
        plt.grid(True)
        plt.show()


    def plot_manipulator_3d(self, theta0, theta1, theta2, theta3):
        # Base coordinates
        x0, y0, z0 = 0, 0, 0
        
        x1 = self.l1 * np.cos(theta0) * np.cos(theta1)
        y1 = self.l1 * np.sin(theta0) * np.cos(theta1)
        z1 = self.l1 * np.sin(theta1)
        
        x2 = x1 + self.l2 * np.cos(theta0) * np.cos(theta1 + theta2)
        y2 = y1 + self.l2 * np.sin(theta0) * np.cos(theta1 + theta2)
        z2 = z1 + self.l2 * np.sin(theta1 + theta2)

        # Corrected third joint position
        x3 = x2 + self.l3 * np.cos(theta0) * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + self.l3 * np.sin(theta0) * np.cos(theta1 + theta2 + theta3)
        z3 = z2 + self.l3 * np.sin(theta1 + theta2 + theta3)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Plotting the links
        ax.plot([x0, x1], [y0, y1], [z0, z1], 'ro-')  # Link 1
        ax.plot([x1, x2], [y1, y2], [z1, z2], 'go-')  # Link 2
        ax.plot([x2, x3], [y2, y3], [z2, z3], 'bo-')  # Link 3
        ax.plot([x3], [y3], [z3], 'ko')  # End-effector
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3-Link Manipulator in 3D Space')
        
        # Set equal aspect ratio for all axes
        ax.set_box_aspect([1,1,1])  # Alternatively, use set_aspect('auto') or 'equal' based on your version
        
        plt.show()

    def point_to_angles(self,x,y,z):
        theta0 = np.arctan(y/x) # angle from positive x axis of base joint
        
        a_target = x/np.cos(theta0)
        b_target = z

        theta1, theta2, theta3 = self.__solve_3link_manipulator(a_target, b_target, self.l1, self.l2, self.l3)

        return theta0, theta1, theta2, theta3



inverse_kinematics = Moveo_IK()

angles = inverse_kinematics.point_to_angles(6,3,5)
