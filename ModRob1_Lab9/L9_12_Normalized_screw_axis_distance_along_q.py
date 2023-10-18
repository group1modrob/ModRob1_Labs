import numpy as np

# input Sq here
Sq = np.array([[1],[2],[2],[4],[5],[6]])

# normalize screw S
norm = np.linalg.norm(Sq[:3],'fro')
Sw = Sq[:3]/norm

# Calculate the distance alone screw axis
Sv = np.array(Sq[3:])
Squ_Sv =np.square(Sv)
Distance_alone_q = np.sqrt(np.sum(Squ_Sv))

# extract the data 
print('Normalized screw axis is ',Sw)
print('Distance traveled along the screw q =',Distance_alone_q)
