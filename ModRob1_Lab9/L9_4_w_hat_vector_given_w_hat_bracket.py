"""
This code snippet was developed by Wilfredo. 
"""
import numpy as np

# Define your w_hat_bracket
w_hat_bracket = np.array([[0, -1, 1], [1, 0, -1], [-1, 1, 0]])

# Obtain the elements of w_hat
w1 = w_hat_bracket[2][1]
w2 = w_hat_bracket[0][2]
w3 = w_hat_bracket[1][0]
w_hat = np.array([w1, w2, w3])

# Print your result
print(f"\nYour w_hat is: {w_hat}")
