import numpy as np

# input homogeneous transformation matrix here
T = np.array([[0,-1,0,3],[0,0,-1,0],[1,0,0,0],[0,0,0,1]])

# extract roation matrix and position vector
# Rotation_matrix
R = np.array([T[0,:3],T[1,:3],T[2,:3]])

# Position vector
P = np.array([[T[0,3]],[T[1,3]],[T[2,3]]])

# Skew-symmetric matrix of position vector
P_skew = np.array([[0, -P[2,0],P[1,0]],[P[2,0],0,-P[0,0]],[-P[1,0],P[0,0],0]])
print(P_skew)

# rotation matrix multipy position matrix
RP = np.matmul(R,P_skew)
#print(RP)

# combining rotation matrix and zeros for Adt matrix
S = (3,3)
R_and_zeros = np.append(R, np.zeros(S),axis =1)

# combining Skew-symmetric matrix of position vector and rotaion matrix
P_and_R = np.append(RP,R,axis=1)

# combining the previous two to get Adt
Adt = np.vstack((R_and_zeros,P_and_R))

# print the result
print('Your adjoint representation is',Adt)
