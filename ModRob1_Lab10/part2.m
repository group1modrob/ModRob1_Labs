
% Theta angles
t1 = 0;
t2 = 0;
t3 = -pi/2;
t4 = pi/2;
q = [t1;t2;t3;t4;];
​
% Robot dimensions
H1 = 89.45;
H2 = 100;
L1 = 35;
L2 = 100;
L3 = 107.6;
​
% {a} vectors
a = [    0  0      0;
         0  0     H1;
        L1  0  H1+H2;
     L1+L2  0  H1+H2;];
​
% {S_w} vectors
rot = [0  0  1;
       0  1  0;
       0  1  0;
       0  1  0;];
​
% {S_v}
% [0 0 1 0 0 0;
%  0 1 0 H1 0 0;
%  0 1 0 H1+H2 0 -L1;
%  0 1 0 H1+H2 0 -L1-L2;]
​
% joint types
jt = 'RRRR';
​
% transformation matrix of end-effector
M = [1 0 0 L1+L2+L3;
     0 1 0        0;
     0 0 1    H1+H2;
     0 0 0        1;];
​
% Function call
[R,p] = FK_PoE(q,a,rot,jt,M)