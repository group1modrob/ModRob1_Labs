"""
This code snippet was developed by Tai. 
"""
import numpy as np
def s_and_q_given_Sq(S_q):
    # normalize screw S
    norm = np.linalg.norm(S_q[:3],'fro')
    Sw = S_q[:3]/norm

    # Calculate the distance alone screw axis
    Sv = np.array(S_q[3:])/norm
    Squ_Sv =np.square(Sv)
    Distance_alone_q = np.sqrt(np.sum(Squ_Sv))

    S=np.concatenate([Sw, Sv])
    return S, Distance_alone_q, Sw, Sv


# input Sq here
Sq = np.array([[1],[2],[2],[4],[5],[6]])
#extract data
s, distance_alone_q, Sw, Sv=s_and_q_given_Sq(Sq)
print('Normalized screw axis is ',s)
print('Distance traveled along the screw q =',distance_alone_q)
