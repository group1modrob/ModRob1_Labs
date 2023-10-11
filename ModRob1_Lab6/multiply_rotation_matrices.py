# Import libraries
import numpy as np

# Define each of the angles first IN RADIANS
theta1 = 90*np.pi/180
theta2 = -45*np.pi/180
theta3 = 0*np.pi/180
theta4 = 45*np.pi/180

# Define the rotation matrices for all the frames
r01 = np.array([[np.cos(theta1), -np.sin(theta1), 0], [np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])
r12 = np.array([[np.cos(theta2), 0, np.sin(theta2)], [0, 1, 0], [-np.sin(theta2), 0, np.cos(theta2)]])
r23 = np.array([[np.cos(theta3), 0, np.sin(theta3)], [0, 1, 0], [-np.sin(theta3), 0, np.cos(theta3)]])
r34 = np.array([[np.cos(theta4), 0, np.sin(theta4)], [0, 1, 0], [-np.sin(theta4), 0, np.cos(theta4)]])
r45 = np.identity(3)

# Tool frame w.r.t. the base frame
r05 = np.matmul(np.matmul(np.matmul(np.matmul(r01, r12), r23), r34), r45)

# Print the result!
np.set_printoptions(suppress=True)
print(r05)

