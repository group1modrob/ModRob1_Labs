import numpy as np

def inverse_homogeneous_matrix(T):
    assert T.shape == (4, 4), "Matrix T must be of shape (4, 4)"
    
    R = T[:3, :3]
    p = T[:3, 3]
    
    R_transposed = R.T
    p_new = -np.dot(R_transposed, p)
    
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_transposed
    T_inv[:3, 3] = p_new
    
    return T_inv

# Sample Input for homogeneous transformation matrix T
T_sample_for_inverse = np.array([
    [0.5, -0.866, 0, 5],
    [0.866, 0.5, 0, 6],
    [0, 0, 1, 7],
    [0, 0, 0, 1]
])

# Compute the inverse for the sample T
T_sample_inverse = inverse_homogeneous_matrix(T_sample_for_inverse)
print(T_sample_inverse)
