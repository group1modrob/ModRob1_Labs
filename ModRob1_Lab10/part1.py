import numpy as np

np.set_printoptions(suppress=True)

"""
USER INPUTS
"""

# For the lab example
# t1 = 0
# t2 = 0
# t3 = -np.pi/2
# t4 = np.pi/2

# For our custom pose
t1 = 15*np.pi/180
t2 = 15*np.pi/180
t3 = 30*np.pi/180
t4 = 0

# Define the robot properties
theta= np.array([[t1],[t2],[t3],[t4]])

H1 = 89.45/1000
H2 = 100/1000
L1 = 35/1000
L2 = 100/1000
L3 = 107.6/1000

a = np.array([
    [0, 0, 0],
    [0, 0, H1],
    [L1, 0, H1+H2],
    [L1+L2, 0, H1+H2]
])

rot = np.array([
    [0, 0, 1],
    [0, 1, 0],
    [0, 1, 0],
    [0, 1, 0]
])

M = np.array([
    [1, 0, 0, L1+L2+L3],
    [0, 1, 0, 0],
    [0, 0, 1, H1+H2],
    [0,0, 0, 1]
])

jt = 'RRRR'


"""
PoE Function
"""
def FK_PoE(theta_val, a_val, rot_val, jt_val):
    #Indentity Matrix
    T=np.eye(4)
    #Number of joints
    n=max(theta_val.shape)
    for ii in range(n-1, -1, -1):
        if jt_val[ii] == 'R':
            rot_hat = np.array([[0, -rot_val[ii,2], rot_val[ii, 1]],
                                [rot_val[ii, 2], 0, -rot_val[ii,0]],
                                [-rot_val[ii, 1], rot_val[ii, 0], 0]])
            e_rot_hat=np.eye(3)+rot_hat*np.sin(theta_val[ii])+(rot_hat@rot_hat)*(1-np.cos(theta_val[ii]))
        elif jt_val[ii] == 'P':
            rot_hat = np.zeros((3, 3))
            e_rot_hat=np.eye(3)
        
        if jt_val[ii] == 'R' and ii > 0:
            Sv = np.cross(rot_val[ii, :], a_val[ii, :])
            Sv = -Sv.T
        elif jt_val[ii] == 'R' and ii == 0:
            Sv = np.array([0, 0, 0])
        elif jt_val[ii] == 'P':
            Sv = a_val[ii, :]
        t=(np.eye(3)*theta_val[ii]+(1-np.cos(theta_val[ii]))*rot_hat+(theta_val[ii]-np.sin(theta_val[ii]))*(rot_hat@rot_hat))@Sv
        t_2=np.array([[t[0]], [t[1]], [t[2]]])

        tee=np.hstack([e_rot_hat,t_2])

        row=np.array([0, 0, 0, 1])
        
        e_zai=np.vstack([tee,row])
        T=e_zai@T
    return T

"""
Actually running the code!
"""
T= FK_PoE(theta, a, rot, jt)

Tf = T @ M
R = Tf[0:3, 0:3]
p = Tf[0:3, 3]

print(f"R = \n{R}")
print(f"\np = \n{p}")