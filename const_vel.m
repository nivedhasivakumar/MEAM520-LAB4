%% Evaluation 1 section
lynxStart();

for t = 1:0.5:100
   q = [pi/6*t pi/6 pi/6 pi/6 pi/6 10];
   [X, T] = updateQ(q);
   lynxServoSim(q);
end

%% Evaluation 3 section
qinit = 0;
joint_vel = [1 1 1 1 1 1];
lynxStart();

for t = 1:0.1:100       
   q = qinit + joint_vel*t; 
   x = X(6,1);
   y = X(6,2);
   z = X(6,3);
   scatter3(x, y, z);
   hold on;
   lynxServoSim(q);
end


%% Evaluation 4

lynxStart();
q = [0 0 -pi/2 0 0 0]; 
body_velocity = [1 1 1 1 1 1];
qdot = ik_velocity(q, body_velocity);

for t = 1:0.1:100
   q = q + qdot*t;
   
   [X, T] = updateQ(q);
   x = X(6,1);
   y = X(6,2);
   z = X(6,3);
   
   scatter3(x, y, z);
   hold on;
   lynxServoSim(q);
   
   J = jacobian(q,5);
   qdot = pinv(J)*body_velocity;
end

