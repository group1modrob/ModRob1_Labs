"""
This code snippet was developed by Wilfredo. 
"""
import numpy as np

# Define your vector w_hat
w_hat = np.array([1, 2, 3])

# The skew-symmetric matrix representation is:
w_hat_skew = np.array([[0, -w_hat[2], w_hat[1]], [w_hat[2], 0, -w_hat[0]], [-w_hat[1], w_hat[0], 0]])

# Print your result
print("Your skew-symmetric matrix representation of w_hat is = ")
print(w_hat_skew)