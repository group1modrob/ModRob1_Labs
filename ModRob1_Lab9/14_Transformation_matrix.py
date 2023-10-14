import numpy as np

#input the six-vector screw axis and q

S = np.array([[0],[0.447],[0.8944],[1.345],[0],[0]])
q = 2.23


Sw = np.array([[0,-S[2,0],S[1,0]],[S[2,0],0,-S[0,0]],[-S[1,0],S[0,0],0]])
Sv = np.array([[S[3,0]],[S[4,0]],[S[5,0]]])

e_Sw = np.array(np.identity(3)+(np.sin(q)*Sw) + (1-np.cos(q))*(np.matmul(Sw,Sw)))

G = np.array(np.matmul((np.identity(3)*q)+((1-np.cos(q))*Sw)+((q-np.sin(q))*(np.matmul(Sw,Sw))),Sv))

T = np.array([[e_Sw[0,0],e_Sw[0,1],e_Sw[0,2],G[0,0]],[e_Sw[1,0],e_Sw[1,1],e_Sw[1,2],G[1,0]],[e_Sw[2,0],e_Sw[2,1],e_Sw[2,2],G[2,0]],[0,0,0,1]])

# print the result
print('Your transformation matrix is ',T)
