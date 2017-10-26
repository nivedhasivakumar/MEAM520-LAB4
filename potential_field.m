%enter start point and end point in q space
qstart=[0,0,0,0,0,1];
qend=[1,1,0,0,0,1];
[XStart,T1]=updateQ(qstart);
[Xend,T2]=updateQ(qend);
[qpath]=potential_field_path(XStart,Xend);
[s,x]=size(qpath);
for j=1:s
    
end

function [qpath]=potential_field_path(start_pos,end_pos)


% influence of the end point on the start
zeta=1;
%influence of the obstacles
eta1=1;
%influence of the obstacles
eta2=1;
%influence of the obstacles
eta3=1;
%influence of the obstacles
eta4=1;
%region of influence obstacle
roi1=1;
%region of influence obstacle
roi2=1;
%region of influence obstacle
roi3=1;
%region of influence obstacle
roi4=1;
%number of itteration
numberofitteration=1000;
%steps size
alpha=10;
%radius of the robot joint
rrobot=10;
%radius of obstacle
robs1=1;
%xyz pos of the obstacle
pobs1=[1;1;1];

robs2=1;
%xyz pos of the obstacle
pobs2=[1;1;1];

robs3=1;
%xyz pos of the obstacle
pobs3=[1;1;1];

robs4=1;
%xyz pos of the obstacle
pobs4=[1;1;1];
%nearness to the final pos
epsilon=1;
%total_path
qpath=zeros(6,3,1);
%need to change this
Jv=[1;1;1];

qpath=start_pos;
pf=end_pos;
probot=qpath(:,:,end);

while(norm(probot(6,:)-pf(6,:))>epsilon)
    for i=1:6
        probotpoint=probot(i,:);
        pfpoint=pf(i,:);
        probotpoint=probotpoint';
        pfpoint=pfpoint';
    % equation 5.4
    Fa=-zeta*(probotpoint-pfpoint);
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
       Fr4=eta1*((1/dobs4)-(1/roi4))*(1/dobs4^2)*(probotpoint-pobs4)/norm(probotpoint-pobs4);
   end
   taur1=Jv'*Fr1;
   taur2=Jv'*Fr2;
   taur3=Jv'*Fr3;
   taur4=Jv'*Fr4;
   tau=taua+taur1+taur2+taur3+taur4;
   probotpoint=probotpoint+alpha*tau/norm(tau);
   probot(i,:)=probotpoint';
   end
   qpath(:,:,end+1)=probot;
end
end