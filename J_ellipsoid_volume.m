% calculate manipulability ellipsoid volume of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function mu_3 = J_ellipsoid_volume(robotRBT, configuration, lastJointIndex)
    J = J_space(robotRBT, configuration, lastJointIndex);
    J_omega = J(1:3, 1:end);
    J_nu = J(4:6, 1:end);
    A_omega = J_omega * J_omega.';
    A_nu = J_nu * J_nu.';

    mu_3.Angular = sqrt(det(A_omega));
    mu_3.Linear = sqrt(det(A_nu));
end
