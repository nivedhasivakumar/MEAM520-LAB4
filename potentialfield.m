 
%initiation of all the matrices that will be used in the code

Int1 = randi([1 36306],1,1);
Int2 = randi([1 36306],1,1);
 
qi = [0,0,0,0,0,1];
%  qf = qfree(Int2,:);
qf=[0,1,0,0,0,1];
jointdistance = zeros();
Fa =zeros();
Fr=zeros();
pjointhistory = zeros();
d =  zeros();
count =1;
jv=zeros();
lynxStart()

Xi = updateQ(qi);
Xf = updateQ(qf);
%plot them
scatter3(Xi(6,1),Xi(6,2),Xi(6,3),'*','black')
hold on;
scatter3(Xf(6,1),Xf(6,2),Xf(6,3),'*','black')
hold on;
%declare the details of obstacles
    pobs1 = [60;-300;170];
    pobs2 = [60;100;150];
    pobs3 = [210;-200;120];
    pobs4 = [210;100;150];
    rrobot=25;
    robs1 = 50;
    robs2 = 50;
    robs3 = 50;
    robs4 = 50;
    rho1=80;
    rho2=80;
    rho3=80;
    rho4=80;



while( norm(Xi - Xf) >10)
%    for a=1:2
%Xi = updateQ(qi);
% d(1) = norm(qi(1) - qf(1));
% 
% scatter3(Xi(6,1),Xi(6,2),Xi(6,3),'.','red')
% hold on;
% 
%         
%     lynxServoSim(qi);
%     pause(0.1);
%     hold on;
%     
%     %attractive force
% Fa = -1*norm(qi(1) - qf(1));
% jv = iwantonlyjv(qi,1);
% taua = jv'*Fa;
% 
%  tau = taua;
%  taut=tau';
%  
%  
%  qi(1) = qi(1) + 0.1*taut(1,1)/norm(tau);
%  qi(2) = qi(2) + 0.1*taut(2,1)/norm(tau);
%  qi(3) = qi(3) + 0.1*taut(3,1)/norm(tau);
%  qi(4) = qi(4);
%  qi(5) = qi(5);
%  qi(6) = 0;
%for a =1:6
    for a =1:6
Xi = updateQ(qi);
Xf = updateQ(qf);

%compute the joint distance from its respective initial origin position to its final origin
%position
jointdistance = [Xi(a,1),Xi(a,2),Xi(a,3);Xf(a,1),Xf(a,2),Xf(a,3)];
d(a) = pdist(jointdistance,'euclidean');
pjoint = [Xi(a,1);Xi(a,2);Xi(a,3)];
scatter3(Xi(6,1),Xi(6,2),Xi(6,3),'.','red')
hold on;

        
    lynxServoSim(qi);
    hold on;
    
    %attractive force
%     if norm(Xi(a,:) - Xf(a,:))>30
%         Fa = -1*[(Xi(a,1)-Xf(a,1))/norm(Xi(a,:) - Xf(a,:));(Xi(a,2)-Xf(a,2))/norm(Xi(a,:) - Xf(a,:));(Xi(a,3)-Xf(a,3))/norm(Xi(a,:) - Xf(a,:))];
%     else
Fa = -1*[Xi(a,1)-Xf(a,1);Xi(a,2)-Xf(a,2);Xi(a,3)-Xf(a,3)];
%     end
Jtot = jacobian(qi,a);
jv=Jtot(1:3,:);
        
taua = jv'*Fa;
%if(((Xi(a,1)-60)^2+(Xi(a,2)+300)^2+(Xi(a,3)-170)^2)<80^2 || ((Xi(a,1)-60)^2+(Xi(a,2)-100)^2+(Xi(a,3)-150)^2)<80^2 || ((Xi(a,1)-210)^2+(Xi(a,2)+200)^2+(Xi(a,3)-120)^2)<80^2 || ((Xi(a,1)-210)^2+(Xi(a,2)-100)^2+(Xi(a,3)-150)^2)<80^2)
 dobs1 = norm(pjoint - pobs1) - rrobot-robs1;
 dobs2 = norm(pjoint - pobs2) - rrobot-robs2;
 dobs3 = norm(pjoint - pobs3) - rrobot-robs3;
 dobs4 = norm(pjoint - pobs4) - rrobot-robs4;
 
 %repulsive forces
 if (dobs1>rho1)
     Fr1 = [0;0;0];
 else 
     Fr1 = 1*((1/dobs1) - (1/rho1))*(1/dobs1^2)*(pjoint-pobs1)/norm(pjoint-pobs1);
 end
 if (dobs2>rho2)
     Fr2 = [0;0;0];
 else 
     Fr2 = 1*((1/dobs2) - (1/rho2))*(1/dobs2^2)*(pjoint-pobs2)/norm(pjoint-pobs2);
 end
 if (dobs3>rho3)
     Fr3 = [0;0;0];
 else 
     Fr3 = 1*((1/dobs3) - (1/rho3))*(1/dobs3^2)*(pjoint-pobs3)/norm(pjoint-pobs3);
 end
 if (dobs4>rho4)
     Fr4 = [0;0;0];
 else 
     Fr4 = 1*((1/dobs4) - (1/rho4))*(1/dobs4^2)*(pjoint-pobs4)/norm(pjoint-pobs4);
 end
 taur1 = jv'*Fr1;
 taur2 = jv'*Fr2;
 taur3 = jv'*Fr3;
 taur4 = jv'*Fr4;
 %total torque on a particular joint
 tau = taua+taur1+taur2+taur3+taur4;
 taut(a,:)=tau';
 
%  qi(1) = qi(1) + 0.1*taut(1)/norm(tau);
%  qi(2) = qi(2) + 0.1*taut(2)/norm(tau);
%  qi(3) = qi(3) + 0.1*taut(3)/norm(tau);
%  qi(4) = qi(4) + 0.1*taut(4)/norm(tau);
%  qi(5) = qi(5) + 0.1*taut(5)/norm(tau);
%  qi(6) = 0;
     end
    tautf=taut(1,:) +taut(2,:) +taut(3,:) + taut(4,:) + taut(5,:) + taut(6,:);
 qi(1) = qi(1) + 0.1*tautf(1)/norm(tau);
 qi(2) = qi(2) + 0.1*tautf(2)/norm(tau);
 qi(3) = qi(3) + 0.1*tautf(3)/norm(tau);
 qi(4) = qi(4) + 0.1*tautf(4)/norm(tau);
 qi(5) = qi(5) + 0.1*tautf(5)/norm(tau);
 qi(6) = 0;
end





