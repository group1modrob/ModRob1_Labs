"""
This code snippet was developed by Wilfredo. 
"""
import numpy as np

# Define your rotation matrix R
R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

# Obtain the inverse (for rotation matrices Rinverse = Rtranspose)
R_inv = R.T

# Print your result
print("\nR_inv = ")
print(R_inv)
print("\n")