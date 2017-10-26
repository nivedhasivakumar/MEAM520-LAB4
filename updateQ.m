% Fill in this function with the mathematics for computing the positions of
% each of the joints of the Lynx robot and of the gripper.
%
% input: q = 1x6 vector [q1, q2, q3, q4, q5, q6] of joint angles (q1-q5)
%            and grip distance (q6)
% output: X = 6x3 matrix of values [x1 y1 z1; x2 y2 z2; ...] containing the
%             positions of each of the joints and the center of the gripper
%             got the given input q
%         T = 4x4x6 matrix of transformation matrices, where each slice
%             T(:,:,i) is the coordinate transformation between links i-1
%             and i
%

function [X, T] = updateQ(q)

T = zeros(4,4,6);
%dh parameters
F = [0,pi/2,76.2,q(1,1);
     146.05,0,0,q(1,2)+(pi/2);
     184.15,0,0,q(1,3)-(pi/2);
     0,pi/2,0,q(1,4)+(pi/2);
     0,0,89,q(1,5);
     20,0,0,0];
X=zeros(6,3);

 
for i = 1:6
% DH homogenous matrix   
    C =([cos(F(i,4)) -sin(F(i,4))*cos(F(i,2)) sin(F(i,4))*sin(F(i,2)) F(i,1)*cos(F(i,4));
             sin(F(i,4)) cos(F(i,4))*cos(F(i,2)) -cos(F(i,4))*sin(F(i,2)) F(i,1)*sin(F(i,4));
             0 sin(F(i,2)) cos(F(i,2)) F(i,3);
             0 0 0 1]);
     T(:,:,i) = C;
        
end
% use the T matrix to calculat X values
a1=T(:,:,1);
a2=a1*T(:,:,2);
a3=a2*T(:,:,3);
a4=a3*T(:,:,4);
a5=a4*T(:,:,5);
v(1,:)=zeros(1,4);
v(2,:)=a1(:,4)';
v(3,:)=a2(:,4)';
v(4,:)=a3(:,4)';
v(5,:)=a4(:,4)';
v(6,:)=a5(:,4)';
v(:,4)=[];
X=v;


end