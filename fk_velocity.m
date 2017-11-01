q = [1 0.3 1 0.1 .5];
joint_vel = [2 3 4 1 2];
velocity = find_fk(q, joint_vel)

function velocity = find_fk(q, joint_vel)
% Call the jacobian function to retrieve the jacobian matrix for robot's
% current pose
J = jacobian(q)

% Is qdot the same as q in body_vel = J(q)*qdot? Is this correct?
velocity = J*joint_vel';
end