% calculate body form FK of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function T = FK_body(robotRBT, configuration, lastJointIndex)
    % calculate M matrix
    theta_0 = configZero(robotRBT,lastJointIndex);
    M = getTransform(robotRBT, theta_0, robotRBT.BodyNames{lastJointIndex});

    % get screw axes
    t = getTransform(robotRBT, theta_0, robotRBT.BodyNames{lastJointIndex});
    lastRot = eye(3);
    screwAxes = {};
    for i = 1:lastJointIndex
        omega = robotRBT.Bodies{1,i}.Joint.JointAxis;
        % apply joint to parent transformation
        jointToParentRot = robotRBT.Bodies{1,i}.Joint.JointToParentTransform(1:3, 1:3);
        omega = jointToParentRot * lastRot * omega.';
        % convert to be wrt body frame
        omega = t(1:3, 1:3) * omega;
        lastRot = jointToParentRot * lastRot;
        % this is one of the main reasons matlab sucks...
        temp = getTransform(robotRBT, theta_0, robotRBT.BodyNames{i}, robotRBT.BodyNames{lastJointIndex});
        q = temp(1:3, end);
        screwAxes{i} = [omega; cross(-omega, q)];
    end

    % calculate forward kinematics using body form of the exponential products
    T = eye(4);
    for i = 1:lastJointIndex
        omega = screwAxes{i}(1:3);
        v = screwAxes{i}(4:6);
        Omega = [0, -omega(3), omega(2);
                omega(3), 0, -omega(1);
                -omega(2), omega(1), 0];
        Omega = [Omega, v; 0 0 0 0];
        expScrewTheta = expm(Omega * configuration(i).JointPosition);
        T = T * expScrewTheta;
    end
    T = M * T;

    % graphically show the defined frames and screw axes
    figure();
    hold on;
    % show robot
    show(robotRBT, configuration);
    % show screw axes
    showScrewAxes(robotRBT, configuration, lastJointIndex);
    % show reference frame
    t = getTransform(robotRBT, configuration, robotRBT.BodyNames{lastJointIndex});
    showFrame(t, 0.5);
    hold off;
end