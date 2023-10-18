# Import
import numpy as np
from L9_6_w_hat_and_theta_given_R import w_hat_and_theta_given_R

# Sample matrix defined by the user
T_sample = np.array([
    [0, 1, 0, 5],
    [0, 0, 0, 6],
    [1, 0, -1, 0],
    [0, 0, 0, 1]
])

# Extract R
R = T_sample[:3, :3]
p = T_sample[:3, 3]

# Calculate the trace (diagonal)
trace_value = np.trace(R)

# Calculate the angle theta
theta = np.arccos((trace_value - 1) / 2.0)

# Scenario 2: theta = k*pi AND k is an EVEN integer
if theta/np.pi % 2 == 0:
    # There is no rotation so:
    Sw = np.array([0, 0, 0])

    # Sv will be:
    Sv = p/np.linalg.norm(p)

    # S will finally be:
    S = np.array([Sw, Sv])

    print(f"\nS = {S}")
    print(f"q = {theta}")

# Scenario 3: theta = k*pi AND k is an ODD integer
elif theta/np.pi % 2 != 0:
    # Get w_hat from R
    w1_hat = np.sqrt((R[0][0] + 1)/2)
    w2_hat = np.sqrt((R[1][1] + 1)/2)
    w3_hat = np.sqrt((R[2][2] + 1)/2)
    w_hat = np.array([w1_hat, w2_hat, w3_hat])
    w_hat_bracket = np.array([[2*np.power(w1_hat, 2)-1, 2*w1_hat*w2_hat, 2*w1_hat*w3_hat],
                    [2*w1_hat*w2_hat, 2*np.power(w2_hat, 2)-1, 2*w2_hat*w3_hat], 
                    [2*w1_hat*w3_hat, 2*w2_hat*w3_hat, 2*np.power(w3_hat, 2)-1]])
    
    # w_hat is Sw
    Sw = w_hat
    Sw_bracket = w_hat_bracket
    Sv = (1/theta) * np.eye(3) - 0.5 * Sw_bracket + ((1/theta - 0.5/np.tan(theta/2)) * (w_hat @ Sw_bracket)) @ p
    S = np.array([Sw, Sv])
    print(f"\nS = {S}")
    print(f"q = {theta}")