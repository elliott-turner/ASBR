% calculate space form FK of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function fkSpace = FK_space(robotRBT, configuration, lastJointIndex, showVisualization)
    % calculate M matrix
    theta_0 = configZero(robotRBT, lastJointIndex);
    M = getTransform(robotRBT, theta_0, robotRBT.BodyNames{lastJointIndex});

    % get screw axes
    lastRot = eye(3);
    screwAxes = {};
    for i = 1:lastJointIndex
        omega = robotRBT.Bodies{1,i}.Joint.JointAxis;
        % apply joint to parent transformation
        jointToParentRot = robotRBT.Bodies{1,i}.Joint.JointToParentTransform(1:3, 1:3);
        omega = jointToParentRot * lastRot * omega.';
        lastRot = jointToParentRot * lastRot;
        % this is one of the main reasons matlab sucks...
        temp = getTransform(robotRBT, theta_0, robotRBT.BodyNames{i});
        q = temp(1:3, end);
        screwAxes{i} = [omega; cross(-omega, q)];
    end
    fkSpace.ScrewAxes = screwAxes;

    % calculate forward kinematics using space form of the exponential products
    T = eye(4);
    for i = 1:lastJointIndex
        T = T * expScrewTheta(screwAxes{i}, configuration(i).JointPosition);
    end
    T = T * M;
    fkSpace.T = T;

    if ~exist('showVisualization', 'var')
        return;
    end
    % graphically show the defined frames and screw axes
    figure();
    hold on;
    % shwo robot
    show(robotRBT, configuration);
    % show screw axes
    showScrewAxes(robotRBT, configuration, lastJointIndex);
    % show reference frame
    t = eye(4);
    showFrame(t, 0.5);
    hold off;
end
