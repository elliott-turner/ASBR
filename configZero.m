% get configuration when all angles are zero
function config0 = configZero(robotRBT, lastJointIndex)
    theta = zeros(1, lastJointIndex);
    config0 = thetaToConfiguration(theta, robotRBT);
end
