%% Evaluation 4

lynxStart();
q = [0 0 0 0 0 0]; 
qdot = [1 1 1 1 1];
body_velocity = [0 0 1 0 0 0];
joint_velocity = ik_velocity(q, body_velocity);
qdot = [qdot 0];

for t = 1:0.1:100
   q = q + qdot*0.1;
   
   [X, T] = updateQ(q);
   x = X(6,1)/25.4;
   y = X(6,2)/25.4;
   z = X(6,3)/25.4;
   
   lynxServoSim(q);
   hold on;
   scatter3(x, y, z,'.','blue');
     
   J = jacobian(q,5);
   qdot = ik_velocity(q, body_velocity);
   qdot = horzcat(qdot', 0);
end