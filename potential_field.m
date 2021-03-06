%enter start point and end point in q space
qstart=[0,0,0,0,0,0];

qend=[0.5,0,0,0,0,0];
lynxStart();
hold on;
% [x y z]=sphere;
% x=4*x;
% y=4*y;
% z=y*z;
% surf(x+5,y-8,z-2);
% hold on;
% surf(x+13,y+3,z-5.5);
% surf(x+11,y-6,z+6.5);
% surf(x+5,y+3.5,z+11);

 lynxServo(qstart);
 lynxServo(qend);
[qpath]=potential_field_path(qstart,qend);
qpath
[s,x]=size(qpath);


function [qpath]=potential_field_path(start_pos,end_pos)


% influence of the end point on the start
zeta=1.2;
%influence of the obstacles
eta1=1.2;
%influence of the obstacles
eta2=1.2;
%influence of the obstacles
eta3=1.2;
%influence of the obstacles
eta4=1.2;
%region of influence obstacle
roi1=50;
%region of influence obstacle
roi2=50;
%region of influence obstacle
roi3=50;
%region of influence obstacle
roi4=50;

%steps size
alpha=0.05;
%radius of the robot joint
rrobot=10;


%radius of obstacle
robs1=80;

%xyz pos of the obstacle
pobs1=[60,-300,170];

%radius of obstacle
robs2=80;
%xyz pos of the obstacle
pobs2=[60,100,150];

%radius of obstacle
robs3=80;
%xyz pos of the obstacle
pobs3=[210,-200,120];

%radius of obstacle
robs4=80;
%xyz pos of the obstacle
pobs4=[210,100,250];

%nearness to the final pos
epsilon=0.2;
%total_path
%qpath=zeros(6,3,1);
%need to change this
Jv=[1;1;1];
%q = [1 0.3 1 0.1 .5];
%joint_vel = [1 1 1 1 1];
%velocity = find_fk(q, joint_vel);

qpath=start_pos;
% pf=end_pos;
% probot=qpath(:,:,end);



[Xend,T2]=updateQ(end_pos);
pf=Xend;
qtemp=zeros(1,5);
while(norm(start_pos-end_pos)>epsilon)
    %start_pos=qpath(:,:,end);
    [probot,T1]=updateQ(start_pos);
    for i=1:5
         Jtot=jacobian(start_pos, i);
         Jv=Jtot(1:3,:);
        %Jv=iwantonlyjv(start_pos, i);
        %Jv=Jv(:,i);
        probotpoint=probot(i,:);
        pfpoint=pf(i,:);
        probotpoint=probotpoint';
        pfpoint=pfpoint';
    % equation 5.4
    if norm(probotpoint-pfpoint)>3*epsilon
    Fa=(-zeta*(probotpoint-pfpoint))/norm(probotpoint-pfpoint);    
    else
    Fa=-zeta*(probotpoint-pfpoint);    
    end
    
    %equation 5.8
    %Jv=previous function
    taua=Jv'*Fa;
    
   %obs 1 
   dobs1=norm(probotpoint-pobs1)-rrobot-robs1;
   if(dobs1>roi1)
       Fr1=[0;0;0];
   else
       Fr1=eta1*((1/dobs1)-(1/roi1))*(1/dobs1^2)*(probotpoint-pobs1)/norm(probotpoint-pobs1);
   end
   
   %obs 2 
   dobs2=norm(probotpoint-pobs2)-rrobot-robs2;
   if(dobs2>roi2)
       Fr2=[0;0;0];
   else
       Fr2=eta2*((1/dobs2)-(1/roi2))*(1/dobs2^2)*(probotpoint-pobs2)/norm(probotpoint-pobs2);
   end
   %obs 1 
   dobs3=norm(probotpoint-pobs3)-rrobot-robs3;
   if(dobs1>roi3)
       Fr3=[0;0;0];
   else
       Fr3=eta3*((1/dobs3)-(1/roi3))*(1/dobs3^2)*(probotpoint-pobs3)/norm(probotpoint-pobs3);
   end
   %obs 1 
   dobs4=norm(probotpoint-pobs4)-rrobot-robs4;
   if(dobs1>roi4)
       Fr4=[0;0;0];
   else
       Fr4=eta4*((1/dobs4)-(1/roi4))*(1/dobs4^2)*(probotpoint-pobs4)/norm(probotpoint-pobs4);
   end
   taur1=Jv'*Fr1;
   taur2=Jv'*Fr2;
   taur3=Jv'*Fr3;
   taur4=Jv'*Fr4;
   tau=taua+taur1+taur2+taur3+taur4;
   tautot(:,i)=tau;
   tautot;
   %start_pos=start_pos+alpha*tau/norm(tau);
   %qtemp(1,i)=start_pos(i);
  % qtemp(1,i);
    
   end
   %start_pos=qtemp;
    tautotal=zeros(5,1);
   for i=1:5 
       tautotal(1,1)=tautotal(1,1)+tautot(1,i);
       tautotal(2,1)=tautotal(2,1)+tautot(2,i);
       tautotal(3,1)=tautotal(3,1)+tautot(3,i);
       tautotal(4,1)=tautotal(4,1)+tautot(4,i);
       tautotal(5,1)=tautotal(5,1)+tautot(5,i);
   end
   
   if tautotal(1)==0
   start_pos(1)=start_pos(1);
   else
   start_pos(1)=start_pos(1)+alpha*tautotal(1)/norm(tautotal);
   end
   if tautotal(2)==0
   start_pos(2)=start_pos(2);
   else
   start_pos(2)=start_pos(2)+alpha*tautotal(2)/norm(tautotal);
   end
    if tautotal(3)==0
   start_pos(3)=start_pos(3);
   else
   start_pos(3)=start_pos(3)+alpha*tautotal(3)/norm(tautotal);
    end
    if tautotal(4)==0
   start_pos(4)=start_pos(4);
   else
   start_pos(4)=start_pos(4)+alpha*tautotal(4)/norm(tautotal);
    end
    if tautotal(5)==0
   start_pos(5)=start_pos(5);
   else
   start_pos(5)=start_pos(5)+alpha*tautotal(5)/norm(tautotal);
    end
   start_pos(6)=0;
   start_pos
   qpath(end+1,:)=start_pos;
   lynxServoSim(start_pos(1),start_pos(2),start_pos(3),start_pos(4),start_pos(5),start_pos(6));
end
end