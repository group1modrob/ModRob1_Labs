import numpy as np
def to_homogeneous_coordinates(v):
    assert len(v) == 3, "Input vector must be of length 3"
    
    return np.append(v, 1)

# Sample 3-vector
sample_vector = np.array([2, 3, 4])
homogeneous_vector = to_homogeneous_coordinates(sample_vector)
print(homogeneous_vector)
