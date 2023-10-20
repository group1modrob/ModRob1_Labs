"""
This code snippet was developed by Namrata. 
"""
import numpy as np

def homogeneous_matrix(R, p):
    assert R.shape == (3, 3), "Rotation matrix R must be of shape (3, 3)"
    assert len(p) == 3, "Position vector p must be of length 3"
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

# Sample input
R_sample = np.array([[0, -1, 0],
                     [1,  0, 0],
                     [0,  0, 1]])

p_sample = np.array([1, 2, 3])

T_sample = homogeneous_matrix(R_sample, p_sample)
print(T_sample)
