% keeps tool tip as close as possible to p_goal while never going
% tol away from the goal and obeying joint limits
function result = IK_pa4_a(robotRBT,lastJointIndex,config_a,p_goal,tol,maxIterations)
    i = 1;
    config_b = config_a;
    configurations = {};
    errors = {};
    result.Status = 0;

    while i <= maxIterations
        configurations{i} = config_b;

        % calculate current error
        T = getTransform(robotRBT,config_b,robotRBT.Bodies{lastJointIndex}.Name);
        D = calcError(T, p_goal);
        errors{i} = D;
        if norm(D) <= tol
            result.Status = 1;
            break;
        end

        % compute optimal change in configuration
        J = J_space(robotRBT,config_b,lastJointIndex-1);
        J_a = J(1:3,:);
        J_e = J(4:6,:);
        A = -sk(T(1:3,4))*J_a + J_e;
        b = p_goal - T(1:3,4);
        config_delta = A\b;

        % update configuration and enforce joint limits
        for j = 1:lastJointIndex-1
            config_b(j).JointPosition = config_b(j).JointPosition + config_delta(j);
            limits = robotRBT.Bodies{j}.Joint.PositionLimits;
            if config_b(j).JointPosition > max(limits)
                config_b(j).JointPosition = max(limits);
            elseif config_b(j).JointPosition < min(limits)
                config_b(j).JointPosition = min(limits);
            end
        end
        i = i+1;
    end
    result.Configuration = config_b;
    result.IterationCount = i;
    result.Configurations = configurations;
    result.Errors = errors;
end

% calculate distance between goal and tip
function D = calcError(T, p_goal)
    D = T(1:3,4) - p_goal;
end

% convert vector to skew symmetrtic matrix
function S = sk(v)
    S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
