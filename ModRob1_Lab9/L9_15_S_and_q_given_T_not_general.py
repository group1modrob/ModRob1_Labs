# Import
import numpy as np
from 6_w_hat_and_theta_given_R import w_hat_and_theta_given_R

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
    # We need to find the individual components of [w_hat]
    w1_hat = np.sqrt((R[0][0] + 1)/2)
    w2_hat = np.sqrt((R[1][1] + 1)/2)
    w3_hat = np.sqrt((R[2][2] + 1)/2)
    
    # Go through all the potential combinations of (w1_hat, w2_hat, w3_hat)
    potential_combinations = [[w1_hat_pos1, w2_hat_pos1, w3_hat_pos1],
                              [w1_hat_pos1, w2_hat_pos1, w3_hat_pos2],
                              [w1_hat_pos1, w2_hat_pos2, w3_hat_pos1],
                              [w1_hat_pos1, w2_hat_pos2, w3_hat_pos2],
                              [w1_hat_pos2, w2_hat_pos1, w3_hat_pos1],
                              [w1_hat_pos2, w2_hat_pos1, w3_hat_pos2],
                              [w1_hat_pos2, w2_hat_pos2, w3_hat_pos1],
                              [w1_hat_pos2, w2_hat_pos2, w3_hat_pos2]]

    # Check which are actual solutions
    solutions = []
    for combination in potential_combinations:
        w1_hat = combination[0]
        w2_hat = combination[1]
        w3_hat = combination[2]
        w_hat = np.array([[2*np.power(w1_hat, 2)-1, 2*w1_hat*w2_hat, 2*w1_hat*w3_hat],
                        [2*w1_hat*w2_hat, 2*np.power(w2_hat, 2)-1, 2*w2_hat*w3_hat], 
                        [2*w1_hat*w3_hat, 2*w2_hat*w3_hat, 2*np.power(w3_hat, 2)-1]])
        
        # Check if this is equal to R, and if so, check if it has been added to solutions. If NOT, then add it as a potential solution:
        if np.allclose(R, w_hat, atol=tolerance):
            if [w1_hat, w2_hat, w3_hat] not in solutions:
                solutions.append([w1_hat, w2_hat, w3_hat])
        
        # Stop if two solutions have been found
        if len(solutions) >= 2:
            break
        
    # Save them with the appropriate name
    w_hat1 = solutions[0]
    w_hat2 = solutions[1]

    # Print the result!
    print(f"\nWith theta1 = {theta1} rad, w_hat is = ")
    print(w_hat1)
    print(f"\nWith theta2 = {theta1} rad, w_hat is = ")
    print(w_hat2, "\n")