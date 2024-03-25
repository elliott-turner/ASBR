% uses iterative numerical inverse kinematics algorithm to
% move robot from configuration_a to configuration_b
% joints beyond lastJointIndex are ignored
function result = J_inverse_kinematics(robotRBT, configuration_a, configuration_b, lastJointIndex, angularError, linearError, maxIterations)
    i = 1;
    e = calculateError(...
        FK_space(robotRBT, configuration_a, lastJointIndex).T, ...
        FK_space(robotRBT, configuration_b, lastJointIndex).T);
    while i <= maxIterations
        if norm(e(1:3)) <= angularError && norm(e(4:6)) <= linearError
            result.Configuration = configuration_a;
            result.Status = 1;
            result.IterationCount = i;
            return;
        end
        deltaTheta = pinv(J_space(robotRBT, configuration_a, lastJointIndex)) * e;
        for j = 1:lastJointIndex
            configuration_a(j).JointPosition = configuration_a(j).JointPosition + deltaTheta(j);
        end
        e = calculateError(...
            FK_space(robotRBT, configuration_a, lastJointIndex).T, ...
            FK_space(robotRBT, configuration_b, lastJointIndex).T);
        i = i + 1;
    end
    result.Configuration = configuration_a;
    result.Status = 0;
    result.IterationCount = i;
end
