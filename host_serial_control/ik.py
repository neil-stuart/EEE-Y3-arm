# Optional: support for 3D plotting in the NB
# If there is a matplotlib error, uncomment the next line, and comment the line below it.
# %matplotlib inline
# %matplotlib widget
import matplotlib.pyplot as plt
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils

my_chain = ikpy.chain.Chain.from_urdf_file("./host_serial_control/moveo.URDF")
target_position = [ 0.1, -0.2, 0.1]
print("The angles of each joints are : ", my_chain.inverse_kinematics(target_position))


fig, ax = plot_utils.init_3d_figure()
my_chain.plot(my_chain.inverse_kinematics(target_position), ax, target=target_position)
plt.xlim(-0.1, 0.1)
plt.ylim(-0.1, 0.1)