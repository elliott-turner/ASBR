% calculate angular and linear isotropy of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function mu_1 = J_isotropy(robotRBT, configuration, lastJointIndex)
    J = J_space(robotRBT, configuration, lastJointIndex);
    J_omega = J(1:3, 1:end);
    J_nu = J(4:6, 1:end);
    A_omega = J_omega * J_omega.';
    A_nu = J_nu * J_nu.';

    mu_1.Angular = sqrt(max(abs(A_omega))) / sqrt(min(abs(A_omega)));
    mu_1.Linear = sqrt(max(abs(A_nu))) / sqrt(min(abs(A_nu)));
end
