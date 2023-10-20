"""
This code snippet was developed by Wilfredo. 
"""
import numpy as np

# Define your w_hat_theta
w_hat_theta = np.array([1, 2, 0])

# Get theta in RADIANS
theta = np.linalg.norm(w_hat_theta)

# Get the rotation axis w_hat
w_hat = w_hat_theta/theta

# Print your result
print("\nYour w_hat and theta are:")
print(f"w_hat = {w_hat}")
print(f"theta = {theta}\n")
