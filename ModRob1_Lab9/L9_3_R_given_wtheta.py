"""
This code snippet was developed by Wilfredo. 
"""
import numpy as np

# Set the numpy print preferences
np.set_printoptions(suppress=True)

# Define your theta IN RADIANS and your w_hat unit vector
theta = 120 * np.pi/180

# Define your w_hat
w_hat = np.array([0, 1, 1])
w1 = w_hat[0]
w2 = w_hat[1]
w3 = w_hat[2]

# Check if w_hat is a unit vector, otherwise, turn it into a unit vector!
norm = np.linalg.norm(w_hat)
if norm != 1:
    print("\nWARNING: Your input w_hat is NOT a unit vector! I will turn it into a unit vector for you!")
    w1 = w1/norm
    w2 = w2/norm
    w3 = w3/norm
    w_hat = np.array([w1, w2, w3])
    print(f"Your unit vector is: w_hat = {w_hat}")

# Define w_hat_bracket
w_hat_bracket = np.array([[0, -w3, w2], [w3, 0, -w1], [-w2, w1, 0]])

# Using Rodrigues' Formula and accounting for the different cases
tolerance = 1e-5

# Scenario 1: theta != k*pi
if np.sin(theta) <= (0-tolerance) or (np.sin(theta) >= (0+tolerance)):
    R = np.eye(3) + np.sin(theta)*w_hat_bracket + (1-np.cos(theta))*(w_hat_bracket @ w_hat_bracket)
    print("\ntheta is NOT k*pi, so:")

# Scenario 2: theta = k*pi AND k is an EVEN integer
elif theta/np.pi % 2 == 0:
    R = np.eye(3)
    print("\nSpecial Case! theta = k*pi, and k is an EVEN integer, so:\n")

# # Scenario 3: theta = k*pi AND k is an ODD integer
elif theta/np.pi % 2 != 0:
    print("\nSpecial Case! theta = k*pi, and k is an ODD integer, so:\n")
    R = np.eye(3) + +2*(w_hat_bracket @ w_hat_bracket)

    
print("Your final rotation matrix is = ")
print(R)