import numpy as np
from L9_6_w_hat_and_theta_given_R import w_hat_and_theta_given_R
# Set the numpy print preferences
np.set_printoptions(suppress=True)
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
## Set the numpy print preferences
np.set_printoptions(suppress=True)
# Sample Input for homogeneous transformation matrix T
T_sample = np.array([
    [0, -1, 0, 3],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])
R_matrix=get_R(T_sample)
p_matrix=get_p(T_sample)
theta,omega_hat,omega_hat_bracket=w_hat_and_theta_given_R(R_matrix)
s_w_matrix=omega_hat
s_v_matrix= s_nu(theta, omega_hat_bracket, omega_hat, p_matrix)
S=np.concatenate([s_w_matrix,s_v_matrix])
print("S:")
print(S)
print()
print()
print("q:")
print(theta)
