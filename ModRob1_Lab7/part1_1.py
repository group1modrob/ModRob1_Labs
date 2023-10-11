# Imports
import numpy as np

# Simplify the print outputs
np.set_printoptions(suppress=True)

# Define the angles in degrees and convert them to radians
theta_1 = 90*np.pi/180
theta_2 = -45*np.pi/180
theta_3 = 0*np.pi/180
theta_4 = 45*np.pi/180

# Obtain the skew-symmetric matrices for each of the axes around which rotations happen
z_hat_bracket = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
y_hat_bracket = np.array([[0, 0, 1], [0, 0, 0], [-1, 0, 0]])

# Calculate the rotation matrices using Rodrigue's formula
e_z_hat_bracket_theta_1 = np.identity(3) + np.sin(theta_1)*z_hat_bracket + (1 - np.cos(theta_1))*np.matmul(z_hat_bracket, z_hat_bracket)
e_y_hat_bracket_theta_2 = np.identity(3) + np.sin(theta_2)*y_hat_bracket + (1 - np.cos(theta_2))*np.matmul(y_hat_bracket, y_hat_bracket)
e_y_hat_bracket_theta_3 = np.identity(3) + np.sin(theta_3)*y_hat_bracket + (1 - np.cos(theta_3))*np.matmul(y_hat_bracket, y_hat_bracket)
e_y_hat_bracket_theta_4 = np.identity(3) + np.sin(theta_4)*y_hat_bracket + (1 - np.cos(theta_4))*np.matmul(y_hat_bracket, y_hat_bracket)

# Calculate the final rotation matrix product of exponentials 
R = np.matmul(np.matmul(np.matmul(np.matmul(np.identity(3), e_z_hat_bracket_theta_1), e_y_hat_bracket_theta_2), e_y_hat_bracket_theta_3), e_y_hat_bracket_theta_4)

# Print the final rotation matrix
print(R)

# Question: Is this familiar to you?
print("\nQuestion: Is this familiar to you?")
print("Answer: Yes, this is the same result as R05 from Lab #6. It is the final answer of Step #5. The reason these are the same is because we are rotating around the same axes, with respect to the same reference (itself), the same degrees, in the same order.")