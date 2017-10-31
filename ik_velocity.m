function [qdot] = ik_velocity(q, velocity)

J = jacobian(q);
lin_comb = [J, velocity];

if rank(J) == rank(lin_comb)
    % Finding the pseudo-inverse and finding qdot
    J_pseudo = pinv(Jacobian);
    qdot = J_pseudo * velocity;
    print qdot
else
    print 'Singularity'
end