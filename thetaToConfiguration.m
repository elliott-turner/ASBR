% convert a theta vector to a robot configuration struct

function configuration = thetaToConfiguration(theta, robotRBT)
    % MATLAB sucks sometimes and idk how to make this struct "cleanly"
    configuration = homeConfiguration(robotRBT);
    for i = 1:length(theta)
        configuration(i).JointPosition = theta(i);
    end
end
