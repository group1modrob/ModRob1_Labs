import numpy as np

# input the screw axis here
S = np.array([[0],[0.447],[0.8944],[1.345],[0],[0]])

representation_matrix = np.array([[0,-S[2,0],S[1,0],S[3,0]],[S[2,0],0,-S[0,0],S[4,0]],[-S[1,0],S[0,0],0,S[5,0]],[0,0,0,0]])

# print te result
print('The representation_matrix of the input vector is', representation_matrix)
