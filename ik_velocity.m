function joint_velocity = ik_velocity(q, body_velocity)

J = jacobian(q, 5);
joint_velocity = find_ik(J, body_velocity);

end

function joint_velocity = find_ik(J, final_vel)

% Finding the linear combination of columns of J with velocity
% lin_comb = [J final_vel'];
% if (rank(J) == rank(lin_comb))
% Finding the pseudo-inverse and finding qdot
J_pseudo = pinv(J);
joint_velocity = J_pseudo * final_vel';
% else
%     disp('Singularity')
%     joint_velocity = 0;
% end

end