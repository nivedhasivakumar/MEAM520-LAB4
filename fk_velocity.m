syms cos(theta1) sin(theta1) cos(theta2) sin(theta2) cos(theta3) sin(theta3) cos(theta4) sin(theta4) cos(theta5) sin(theta5) L1 L2 L3 L4 L5 L6 reals

%Frame 1 w.r.t Frame 0
A1 = [cos(theta1)    0  -sin(theta1) 0;
      sin(theta1)    0  cos(theta1)  0;
       0     -1    0  L1;
       0      0    0   1];
          
%Frame 2 w.r.t Frame 1          
A2 = [ sin(theta2)  cos(theta2)  0   L2*cos(theta2);
      -cos(theta2)  sin(theta2)  0   L2*-cos(theta2);
         0    0    1      0;
         0    0    0      1];

%Frame 3 w.r.t Frame 2
A3 = [-sin(theta3) -cos(theta3)  0   L3*-sin(theta3);
      cos(theta3)  -sin(theta3)  0   L3*cos(theta3);
       0      0    1      0;
       0      0    0      1];

%Frame 4 w.r.t Frame 3
A4 = [sin(theta4)   0   cos(theta4)   0;
      -cos(theta4)  0   sin(theta4)   0;
        0   -1    0     0;
        0    0    0     1];
    
%Frame 5 w.r.t Frame 4
A5 = [cos(theta5) -sin(theta5)  0        0;
      sin(theta5)  cos(theta5)  0        0;
        0               0       1  L4 + L5;
        0               0       0        1];
          
%Gripper Frame w.r.t. Frame 5
A6 = [1 0 0  0;
      0 1 0  0;
      0 0 1  L6;
      0 0 0  1];

%Puts the Homogeneous Tranformations in T Matrix
T(:,:,1) = A1;
T(:,:,2) = A2;
T(:,:,3) = A3;
T(:,:,4) = A4;
T(:,:,5) = A5;
T(:,:,6) = A6;
 
% Find final transformation matrix for Lynx
Tnew = A1*A2*A3*A4*A5*A6;
% Holds translations of transformation matrices
translation = Tnew(1:3,4);

%% Finding Jacobian for forward linear velocity kinematics
xdiff = [diff(translation(1),theta1), diff(translation(1),theta2), diff(translation(1),theta3), diff(translation(1),theta4), diff(translation(1),theta5)];
ydiff = [diff(translation(2),theta1), diff(translation(2),theta2), diff(translation(2),theta3), diff(translation(2),theta4), diff(translation(2),theta5)];
zdiff = [diff(translation(3),theta1), diff(translation(3),theta2), diff(translation(3),theta3), diff(translation(3),theta4), diff(translation(3),theta5)];

Jacobian_v = [xdiff; ydiff; zdiff];

% pretty(Jacobian_v)

syms theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) t
theta1(t) = theta1;
theta2(t) = theta2;
theta3(t) = theta3;
theta4(t) = theta4;
theta5(t) = theta5;

body_lin_velocity = Jacobian_v*[diff(theta1(t), t); diff(theta2(t), t); diff(theta3(t), t); diff(theta4(t), t); diff(theta5(t), t)];

%% Finding Jacobian for forward angular velocity kinematics

R_0_1 = A1(1:3,1:3);
R_0_2 = A1(1:3,1:3) * A2(1:3,1:3);
R_0_3 = A1(1:3,1:3) * A2(1:3,1:3) * A3(1:3,1:3);
R_0_4 = A1(1:3,1:3) * A2(1:3,1:3) * A3(1:3,1:3) * A4(1:3,1:3);

z_for_omega = [0; 0; 1]; % z_for_omega is always this value
omega_1 = z_for_omega;
omega_2 = R_0_1 * z_for_omega;
omega_3 = R_0_2 * z_for_omega;
omega_4 = R_0_3 * z_for_omega;
omega_5 = R_0_4 * z_for_omega;

Jacobian_w = [omega_1, omega_2, omega_3, omega_4, omega_5];

% pretty(Jacobian_v)

body_ang_velocity = Jacobian_w*[diff(theta1(t), t); diff(theta2(t), t); diff(theta3(t), t); diff(theta4(t), t); diff(theta5(t), t)];

%% Finding final Jacobian

Jacobian = [Jacobian_v; Jacobian_w];



