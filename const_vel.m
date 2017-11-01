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