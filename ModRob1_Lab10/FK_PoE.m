function [R,p]=FK_PoE(theta,a,rot,jt,M)

% this code caclulate the forward kinematics using the product of
% exponentials PoE formula
% written by Madi Babaiasl

% function inputs:

% theta: vector of joint angles
% a: matrix of 3D arbitrary points on the joint axes at zero configuration 
   % written in coordinates of the fixed base frame where each row belongs 
   % to one point
% rot: matrix of 3D rotational part of the screw axes at zero configuration, where each
% row belongs to one joint and each row is a unit vector. For rotational
% joints this is rotation axis and for prismatic joints, this is
% zero
% jt: a string of letters showing joint types, e.g. 'RRP'
% M: Homogenous transformation matrix of end-effector w.r.t. base at zero configuration

% outputs:

% R is the rotation matrix of the end-effector w.r.t. base
% p is the position of the end-effector w.r.t. base
%-------------------------------------------------------------------------

    T=POE(theta,a,rot,jt);
    Tf=T*M;    

    R=Tf(1:3,1:3);
    p=Tf(1:3,4);

end

% utility function ----------------------------------------------------
function T=POE(theta,a,rot,jt)

    T=eye(4,4);
    
    % number of joints
    n=length(theta);

    for ii=n:-1:1

        if jt(ii)=='R'     
            rot_hat=[0       -rot(ii,3)     rot(ii,2);...
                   rot(ii,3)      0      -rot(ii,1);...
                   -rot(ii,2)    rot(ii,1)      0];        
            e_rot_hat=eye(3,3)+rot_hat*sin(theta(ii))+rot_hat*rot_hat*(1-cos(theta(ii)));
        elseif jt(ii)=='P'
            rot_hat=zeros(3,3);
            e_rot_hat=eye(3,3);
        end
    
        if (jt(ii)=='R') & (ii>1)
            Sv=-transpose(cross(rot(ii,:),a(ii,:)));
        elseif (jt(ii)=='R') & (ii==1)
            Sv=[0;0;0];
        elseif jt(ii)=='P'
            Sv=a(ii,:)';
        end

        t=(eye(3,3)*theta(ii)+(1-cos(theta(ii)))*rot_hat+(theta(ii)-sin(theta(ii)))*rot_hat*rot_hat)*Sv;

        e_zai=[e_rot_hat t;0 0 0 1];
    
        T=e_zai*T;
    
    end

end