% calculate body form jacobian of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function J = J_body(robotRBT, configuration, lastJointIndex)
    J = zeros(6, lastJointIndex);

    fkBody = FK_body(robotRBT, configZero(robotRBT, lastJointIndex), lastJointIndex);

    % calculate exponential twists
    expTwists = cell(1, lastJointIndex);
    for i = 1:lastJointIndex
        expTwists{i} = expScrewTheta(fkBody.ScrewAxes{i}, configuration(i).JointPosition);
    end

    % calculate space form jacobian
    T = eye(4);
    for i = 1:lastJointIndex
        T = T * expTwists{i};
        J(:, i) = adt(T) * fkBody.ScrewAxes{i};
    end
end
