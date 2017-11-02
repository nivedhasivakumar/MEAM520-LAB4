function joint_velocity = ik_velocity(q, body_velocity)
bvel = body_velocity;
J = jacobian(q, 5);
joint_velocity = find_ik(J, bvel);
end

function joint_velocity = find_ik(J, final_vel)
J_pseudo = pinv(J);
joint_velocity = J_pseudo * final_vel';
end