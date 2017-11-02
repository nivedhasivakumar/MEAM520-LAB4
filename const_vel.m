%% Evaluation 1
lynxStart();

for t = 1:0.1:100
   q = [pi/6*t pi/6 pi/6 pi/6 pi/6 10];
   [X, T] = updateQ(q);
   x = X(6,1)/12.5;
   y = X(6,2)/12.5;
   z = X(6,3)/12.5;
   scatter3(x, y, z);
   hold on;
   lynxServoSim(q);
end

%% Evaluation 3

init_q = [pi/4 pi/4 pi/4 pi/4 pi/4 10];

for t = 1:0.1:100
   q = [pi/4*t pi/4*t pi/4*t pi/4*t pi/4*t 10];
   [X, T] = updateQ(q);
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
qdot = [0 0 1 0 0 0];
body_velocity = ik_velocity(q);

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

