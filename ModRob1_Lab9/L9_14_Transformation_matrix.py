import numpy as np

S_sample = np.array([[0],[1/np.sqrt(5)],[2/np.sqrt(5)],[3/np.sqrt(5)],[0],[0]])
q_sample=2.23
#Calculates S_bracket from S_w and S_v from S_sample
def get_s_w_bracket_and_s_v(S):
    s_w = S[:3]
    s_w_bracket=np.array([[0, -s_w[2][0], s_w[1][0]], [s_w[2][0], 0, -s_w[0][0]], [-s_w[1][0], s_w[0][0], 0]])
    s_v = S[3:]
    return s_w_bracket, s_v

s_omega,s_nu=get_s_w_bracket_and_s_v(S_sample)
#Calculates T with rotaitonal and linear part 
def get_T(s_w_bracket,s_v, theta):
    e_s_q=np.eye(3)+(np.sin(theta)*s_w_bracket)+((1-np.cos(theta))*(s_w_bracket @ s_w_bracket))
    g_s_q=((np.eye(3)*theta)+((1-np.cos(theta))*s_w_bracket)+(theta-np.sin(theta))*(s_w_bracket @ s_w_bracket))@s_v
    
    T=np.concatenate((e_s_q,g_s_q),1)
    #Adds the extra row
    row=np.array([0, 0, 0, 1])
    T=np.vstack([T,row])
    return T

print(get_T(s_omega,s_nu,q_sample))
