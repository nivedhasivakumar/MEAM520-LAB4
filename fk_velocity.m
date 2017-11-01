q = [1 0.3 .5 0.1 .5 0];
joint_vel = [2 3 4 0 2 0];
ee_velocity = find_fk(q, joint_vel)

function ee_velocity = find_fk(q, joint_vel)

% Call the jacobian function to retrieve the jacobian matrix for robot's current pose
J = jacobian(q, 5)
ee_velocity = J*joint_vel';

end