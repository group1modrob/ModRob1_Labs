"""
This code snippet was developed by Namrata. 
"""
import numpy as np

def extract_from_homogeneous(T):
    assert T.shape == (4, 4), "Matrix T must be of shape (4, 4)"
    
    R = T[:3, :3]
    p = T[:3, 3]
    return R, p

# Sample Input for homogeneous transformation matrix T
T_demo_sample = np.array([
    [0, -1, 0, 3],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])

# Extract R and p from the demo sample T
R_demo_extracted, p_demo_extracted = extract_from_homogeneous(T_demo_sample)
print(R_demo_extracted)
print(p_demo_extracted)
