% calculate space form FK of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function T = FK_space(robotRBT, configuration, lastJointIndex)
    % calculate M matrix
    theta = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    theta_0 = thetaToConfiguration(theta, robotRBT);
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

    % calculate forward kinematics using space form of the exponential products
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
    T = T * M;

    % graphically show the defined frames and screw axes
    hold on;
    show(robotRBT, configuration);
    % show reference frame
    quiver3(0, 0, 0, 0.5, 0, 0, 'LineWidth', 4, 'Color', 'red');
    quiver3(0, 0, 0, 0, 0.5, 0, 'LineWidth', 4, 'Color', 'green');
    quiver3(0, 0, 0, 0, 0, 0.5, 'LineWidth', 4, 'Color', 'blue');
    % show screw axes
    for i = 1:lastJointIndex
        t = getTransform(robotRBT, configuration, robotRBT.BodyNames{i});
        q_i = t(1:3, end);
        w_i = t(1:3, 1:3) * robotRBT.Bodies{1,i}.Joint.JointAxis.';
        w_i = w_i .* 0.5;
        quiver3(q_i(1), q_i(2), q_i(3), w_i(1), w_i(2), w_i(3), 'LineWidth', 4, 'Color', 'yellow');
    end
    axis equal; % makes axes proportional
    view(3); % force 3D view (fixes "stretching" issues?)
    camlight; % make shaded
    set(findall(gca, 'Type', 'patch'), 'FaceAlpha', 0.5); % make transparent
    axis vis3d; % keeps it from zooming in and out to fit in plot window
    grid on; % background grid
    hold off;
end
