function velocity = fk_velocity(q)
% Call the jacobian function to retrieve the symbolic jacobian matrix
[J_sym, qdot] = jacobian;

% Substitute for following values in the Jacobian matrix
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);
theta4 = q(4);
theta5 = q(5);

% Fill correct values for L1, L2, L3, L4, L5 and L6
L1 = 1;
L2 = 1;
L3 = 1;
L4 = 1;
L5 = 1;
L6 = 1;

syms t reals

velocity = subs(J_sym)*qdot;

end