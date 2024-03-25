% calculate space form jacobian of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function J = J_space(robotRBT, configuration, lastJointIndex)
    J = zeros(6, lastJointIndex);

    fkSpace = FK_space(robotRBT, configZero(robotRBT, lastJointIndex), lastJointIndex);

    % calculate exponential twists
    expTwists = cell(1, lastJointIndex);
    for i = 1:lastJointIndex
        expTwists{i} = expScrewTheta(fkSpace.ScrewAxes{i}, configuration(i).JointPosition);
    end

    % calculate space form jacobian
    T = eye(4);
    for i = 1:lastJointIndex
        T = T * expTwists{i};
        J(:, i) = adt(T) * fkSpace.ScrewAxes{i};
    end
end
