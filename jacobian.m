function J = jacobian(q, n)

% Modify theta names
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);
theta4 = q(4);
theta5 = q(5);
theta6 = q(6);

L1 = 76.2;
L2 = 146.05;
L3 = 184.15;
L4 = 0;
L5 = 89;
L6 = 0;

% syms theta1 theta2 theta3 theta4 theta5 L1 L2 L3 L4 L5 L6 reals

%Frame 1 w.r.t Frame 0
A1 = [cos(theta1)    0  -sin(theta1) 0;
      sin(theta1)    0  cos(theta1)  0;
       0            -1     0        L1;
       0             0     0        1];
          
%Frame 2 w.r.t Frame 1          
A2 = [ sin(theta2)  cos(theta2)  0   L2*cos(theta2);
      -cos(theta2)  sin(theta2)  0   L2*-cos(theta2);
         0              0        1          0;
         0              0        0          1];

%Frame 3 w.r.t Frame 2
A3 = [-sin(theta3) -cos(theta3)  0   L3*-sin(theta3);
      cos(theta3)  -sin(theta3)  0   L3*cos(theta3);
       0                0        1          0;
       0                0        0          1];

%Frame 4 w.r.t Frame 3
A4 = [sin(theta4)   0   cos(theta4)   0;
      -cos(theta4)  0   sin(theta4)   0;
        0           -1        0       0;
        0           0         0       1];
    
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
 
% % % Find final transformation matrix for Lynx
% Tnew = A1;
% 
% % % Holds translations of transformation matrices
% translation = Tnew(1:3,4);

%% Finding Jacobian for forward linear velocity kinematics
% xdiff = [diff(translation(1),theta1)];
% ydiff = [diff(translation(2),theta1)];
% zdiff = [diff(translation(3),theta1)];
% 
% lin_vel = [xdiff; ydiff; zdiff]

if(n == 5)
    xcomp1 = (cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))*(L4 + L5) - L2*cos(theta2)*sin(theta1) - L3*cos(theta2)*cos(theta3)*sin(theta1) + L3*sin(theta1)*sin(theta2)*sin(theta3);
    xcomp2 = - (cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))*(L4 + L5) - L2*cos(theta1)*sin(theta2) - L3*cos(theta1)*cos(theta2)*sin(theta3) - L3*cos(theta1)*cos(theta3)*sin(theta2);
    xcomp3 = - (cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))*(L4 + L5) - L3*cos(theta1)*cos(theta2)*sin(theta3) - L3*cos(theta1)*cos(theta3)*sin(theta2);
    xcomp4 = -(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))*(L4 + L5);
    xcomp5 = 0;
    xdiff = [xcomp1, xcomp2, xcomp3, xcomp4, xcomp5];

    ycomp1 = L2*cos(theta1)*cos(theta2) - (cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))*(L4 + L5) + L3*cos(theta1)*cos(theta2)*cos(theta3) - L3*cos(theta1)*sin(theta2)*sin(theta3);
    ycomp2 = - (cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))*(L4 + L5) - L2*sin(theta1)*sin(theta2) - L3*cos(theta2)*sin(theta1)*sin(theta3) - L3*cos(theta3)*sin(theta1)*sin(theta2);
    ycomp3 = - (cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))*(L4 + L5) - L3*cos(theta2)*sin(theta1)*sin(theta3) - L3*cos(theta3)*sin(theta1)*sin(theta2);
    ycomp4 = -(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))*(L4 + L5);
    ycomp5 = 0;
    ydiff = [ycomp1, ycomp2, ycomp3, ycomp4, ycomp5];

    zcomp1 = 0;
    zcomp2 = L3*sin(theta2)*sin(theta3) - L2*sin(theta2) - L3*cos(theta2)*cos(theta3) - (cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))*(L4 + L5);
    zcomp3 = L3*sin(theta2)*sin(theta3) - L3*cos(theta2)*cos(theta3) - (cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))*(L4 + L5);
    zcomp4 = -(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))*(L4 + L5);
    zcomp5 = 0;
    zdiff = [zcomp1, zcomp2, zcomp3, zcomp4, zcomp5];

elseif (n == 4)
    xcomp1 = L3*sin(theta1)*sin(theta2)*sin(theta3) - L3*cos(theta2)*cos(theta3)*sin(theta1) - L2*cos(theta2)*sin(theta1);
    xcomp2 =  - L2*cos(theta1)*sin(theta2) - L3*cos(theta1)*cos(theta2)*sin(theta3) - L3*cos(theta1)*cos(theta3)*sin(theta2);
    xcomp3 = - L3*cos(theta1)*cos(theta2)*sin(theta3) - L3*cos(theta1)*cos(theta3)*sin(theta2);
    xcomp4 = 0;
    xdiff = [xcomp1, xcomp2, xcomp3, xcomp4, 0];
    
    ycomp1 = L2*cos(theta1)*cos(theta2) + L3*cos(theta1)*cos(theta2)*cos(theta3) - L3*cos(theta1)*sin(theta2)*sin(theta3);
    ycomp2 = - L2*sin(theta1)*sin(theta2) - L3*cos(theta2)*sin(theta1)*sin(theta3) - L3*cos(theta3)*sin(theta1)*sin(theta2);
    ycomp3 = - L3*cos(theta2)*sin(theta1)*sin(theta3) - L3*cos(theta3)*sin(theta1)*sin(theta2);
    ycomp4 = 0;
    ydiff = [ycomp1, ycomp2, ycomp3, ycomp4, 0];
    
    zcomp1 = 0;
    zcomp2 = L3*sin(theta2)*sin(theta3) - L3*cos(theta2)*cos(theta3) - L2*sin(theta2);
    zcomp3 = L3*sin(theta2)*sin(theta3) - L3*cos(theta2)*cos(theta3);
    zcomp4 = 0;
    zdiff = [zcomp1, zcomp2, zcomp3, zcomp4, 0]; 
    
elseif (n == 3)
    xcomp1 = L3*sin(theta1)*sin(theta2)*sin(theta3) - L3*cos(theta2)*cos(theta3)*sin(theta1) - L2*cos(theta2)*sin(theta1);
    xcomp2 = - L2*cos(theta1)*sin(theta2) - L3*cos(theta1)*cos(theta2)*sin(theta3) - L3*cos(theta1)*cos(theta3)*sin(theta2);
    xcomp3 = - L3*cos(theta1)*cos(theta2)*sin(theta3) - L3*cos(theta1)*cos(theta3)*sin(theta2);
    xdiff = [xcomp1, xcomp2, xcomp3, 0, 0];
    
    ycomp1 = L2*cos(theta1)*cos(theta2) + L3*cos(theta1)*cos(theta2)*cos(theta3) - L3*cos(theta1)*sin(theta2)*sin(theta3);
    ycomp2 = - L2*sin(theta1)*sin(theta2) - L3*cos(theta2)*sin(theta1)*sin(theta3) - L3*cos(theta3)*sin(theta1)*sin(theta2);
    ycomp3 = - L3*cos(theta2)*sin(theta1)*sin(theta3) - L3*cos(theta3)*sin(theta1)*sin(theta2);
    ydiff = [ycomp1, ycomp2, ycomp3, 0, 0];
    
    zcomp1 = 0;
    zcomp2 = L3*sin(theta2)*sin(theta3) - L3*cos(theta2)*cos(theta3) - L2*sin(theta2);
    zcomp3 = L3*sin(theta2)*sin(theta3) - L3*cos(theta2)*cos(theta3);
    zdiff = [zcomp1, zcomp2, zcomp3, 0, 0];
    
elseif (n == 2)
    xcomp1 = -L2*cos(theta2)*sin(theta1);
    xcomp2 = -L2*cos(theta1)*sin(theta2);
    xdiff = [xcomp1, xcomp2, 0, 0, 0];
    
    ycomp1 = L2*cos(theta1)*cos(theta2);
    ycomp2 = -L2*sin(theta1)*sin(theta2);
    ydiff = [ycomp1, ycomp2, 0, 0, 0];
    
    zcomp1 = 0;
    zcomp2 = -L2*sin(theta2);
    zdiff = [zcomp1, zcomp2, 0, 0, 0];
    
elseif (n == 1)
    xdiff = [0, 0, 0, 0, 0];   
    ydiff = [0, 0, 0, 0, 0];  
    zdiff = [0, 0, 0, 0, 0];
end

Jacobian_v = [xdiff; ydiff; zdiff];

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

zeros = [0; 0; 0]
if (n == 5)
    Jacobian_w = [omega_1, omega_2, omega_3, omega_4, omega_5];
elseif (n == 4)
    Jacobian_w = [omega_1, omega_2, omega_3, omega_4, zeros];
elseif (n == 3)
    Jacobian_w = [omega_1, omega_2, omega_3, zeros, zeros];
elseif (n == 2)
    Jacobian_w = [omega_1, omega_2, zeros, zeros, zeros];
else
    Jacobian_w = [omega_1, zeros, zeros, zeros, zeros];    
end

%% Finding final Jacobian and body velocity

J = [Jacobian_v; Jacobian_w];
end