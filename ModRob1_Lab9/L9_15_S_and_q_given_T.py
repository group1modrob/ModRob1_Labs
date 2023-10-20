"""
This code snippet was developed by Bryan. 
"""
import numpy as np
from L9_6_w_hat_and_theta_given_R import w_hat_and_theta_given_R

# Set the numpy print preferences
np.set_printoptions(suppress=True)

"""
Defining the functions for use later
"""
# Get R from T Matrix
def get_R(T):
    assert T.shape == (4, 4), "Matrix T must be of shape (4, 4)"
    R = T[:3, :3]
    return R
# Get p from T Matrix
def get_p(T):
    assert T.shape == (4, 4), "Matrix T must be of shape (4, 4)"
    p = T[:3, 3]
    return p
# Get S_nu
def s_nu(theta, omega_hat_bracket, omega_hat, p_matrix):
    s_v = ((1/theta) * np.eye(3) - 0.5 * omega_hat_bracket + ((1/theta - 0.5/np.tan(theta/2)) * (omega_hat_bracket @ omega_hat_bracket))) @ p_matrix
    return s_v

# Function to get S and q
def get_S_and_q(T):
    R_matrix=get_R(T)
    p_matrix=get_p(T)

    # Get theta, w_hat, and w_hat_bracket
    theta,omega_hat,omega_hat_bracket=w_hat_and_theta_given_R(R_matrix)

    # Account for all possible cases

    #SCENARIO 1: sin(theta) != 0
    if np.sin(theta) != 0:
        s_w_matrix=omega_hat
        s_v_matrix= s_nu(theta, omega_hat_bracket, omega_hat, p_matrix)
        S=np.concatenate([s_w_matrix,s_v_matrix])

        return S, theta

    # SCENARIO 2: theta = k*pi AND k is an EVEN integer
    if theta/np.pi % 2 == 0:
        # There is no rotation so:
        Sw = np.array([0, 0, 0])

        # Sv will be:
        Sv = p_matrix/np.linalg.norm(p_matrix)
        d = np.linalg.norm(p_matrix)

        # S will finally be:
        S = np.concatenate([Sw, Sv])

        return S, d

    # Scenario 3: theta = k*pi AND k is an ODD integer
    elif theta/np.pi % 2 != 0:
        # Get w_hat from R
        w1_hat = np.sqrt((R_matrix[0][0] + 1)/2)
        w2_hat = np.sqrt((R_matrix[1][1] + 1)/2)
        w3_hat = np.sqrt((R_matrix[2][2] + 1)/2)
        w_hat = np.array([w1_hat, w2_hat, w3_hat])
        w_hat_bracket = np.array([[2*np.power(w1_hat, 2)-1, 2*w1_hat*w2_hat, 2*w1_hat*w3_hat],
                        [2*w1_hat*w2_hat, 2*np.power(w2_hat, 2)-1, 2*w2_hat*w3_hat], 
                        [2*w1_hat*w3_hat, 2*w2_hat*w3_hat, 2*np.power(w3_hat, 2)-1]])
        
        # w_hat is Sw
        Sw = w_hat
        Sw_bracket = w_hat_bracket
        Sv = ((1/theta) * np.eye(3) - 0.5 * Sw_bracket + ((1/theta - 0.5/np.tan(theta/2)) * (w_hat @ Sw_bracket))) @ p_matrix
        S = np.concatenate([Sw, Sv])
        return S, theta

"""
Code Execution
"""
# Sample Input for homogeneous transformation matrix T
T_sample_rotation_only = np.array([
    [0.3536, 0.5732, -0.7392, 0],
    [0.6124, -0.7392, -0.2803, 0],
    [-0.7071, -0.3536, -0.6124, 0],
    [0, 0, 0, 1]
])
T_sample_translation_only = np.array([
    [1, 0, 0, 10],
    [0, 1, 0, 5],
    [0, 0, 1, 5],
    [0, 0, 0, 1]
])
T_sample = np.array([
    [0, -1, 0, 3],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])

# Change your sample matrix as you desire
S, q = get_S_and_q(T_sample)
print(f"\nS = {S}")
print(f"\nq = {q}")