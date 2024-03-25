% calculate angular and linear condition of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function mu_2 = J_condition(robotRBT, configuration, lastJointIndex)
    mu_1 = J_isotropy(robotRBT, configuration, lastJointIndex);
    mu_2.Angular = mu_1.Angular^2;
    mu_2.Linear = mu_1.Linear^2;
end
