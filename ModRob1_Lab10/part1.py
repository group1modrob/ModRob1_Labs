import numpy as np

np.set_printoptions(suppress=True)

t1 = 0
t2 = 0
t3 = 0
t4 = 0

theta= np.array([[t1],[t2],[t3],[t4]])

H1 = 89.45
H2 = 100
L1 = 35
L2 = 100
L3 = 107.6

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
            print(e_rot_hat)
        elif jt_val[ii] == 'P':
            rot_hat = np.zeros((3, 3))
            e_rot_hat=np.eye(3)
        if jt_val[ii] == 'R' and ii > 1:
            Sv = np.cross(rot_val[ii, :], a_val[ii, :])
            Sv = -Sv.T
        elif jt_val[ii-1] == 'R' and ii == 1:
            Sv = np.array([0, 0, 0])
        elif jt_val[ii-1] == 'P':
            Sv = rot_val[ii, :]
        t=(np.eye(3)*theta_val[ii]+(1-np.cos(theta_val[ii])*rot_hat+(theta_val[ii]-np.sin(theta_val[ii]))*rot_hat@rot_hat))@Sv
    t_2=np.array([[t[0]], [t[1]], [t[2]]])
    tee=np.hstack([e_rot_hat,t_2])
    row=np.array([0, 0, 0, 1])
    e_zai=np.vstack([tee,row])
    T=e_zai@T
    return T
T= FK_PoE(theta, a, rot, jt)