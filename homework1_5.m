%% A

clear
clc
syms theta
syms d L 'positive'

%frame from cylinder to cube
Rbd = [0 sind(60-theta)  cosd(60-theta)
       0 cosd(60-theta)  -sind(60-theta)
       -1 0  0];
   
obd = [L*cosd(theta)
       L*sind(theta)
       0];
   
Abd = [Rbd obd;zeros(1,3) 1];

%frame from cube to gripper
Rde = eye(3);
   
ode = [0
       0
       d];
   
Ade = [Rde ode;zeros(1,3) 1];

%from base to gripper
Tbe = Abd*Ade;

%% B
a1 = sqrt(3)/2 * L;
a2 = 0;

alpha1 = 90;
alpha2 = 0;

d1 = 0;
d2 = d + L/2;

v1 = 30 + theta;
v2 = -90;

DH_params = [a1 alpha1 d1 v1
             a2 alpha2 d2 v2];
%% C

Tbe = subs(Tbe,L,1)

pprime = [1.73
          0
          0
          1];
[thetad dd] = solve(Tbe*[zeros(3,1);1] == pprime,{theta, d},'ReturnConditions','true');


%%
% controll check
    theta = 60 - asind(sind(120)/1.73)

    d = sind(theta)/(sind(120)/1.73)
% end of control check

function T = T_DH(DH_params)
%T_DH - Transformation matrix between two consecutive frames given the DH-parameters
%   
% Inputs:
%    DH_params - nx4 matrix, with each row containing DH parameters in order
%                [a,alpha,d,theta] and n being number of degrees of freedom of robot.
%
% Outputs:
%    T - 4x4 matrix, Transformation between frame i-1 and i given DH-parameters i.

    % TODO T = ...
    
    %for readability reasons
    a = DH_params(:,1);
    alpha = DH_params(:,2);
    d = DH_params(:,3);
    theta = DH_params(:,4);
    
    T = sym(eye(4));
    for i = 1:length(DH_params(:,1))
        
        T = T * [cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i))
                sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i))
                0 sin(alpha(i)) cos(alpha(i)) d(i)
                0 0 0 1];
    end
end