% uses redundancy resolution to avoid singularities and
% move robot from configuration_a to configuration_b
% joints beyond lastJointIndex are ignored
function result = redundancy_resolution(robotRBT, configuration_a, configuration_b, lastJointIndex, angularError, linearError, maxIterations, scale, showVisualization)
    i = 1;
    configurations = {};
    result.Status = 0;
    e = calculateError(...
        FK_space(robotRBT, configuration_a, lastJointIndex).T, ...
        FK_space(robotRBT, configuration_b, lastJointIndex).T);
    while i <= maxIterations
        configurations{i} = configuration_a;
        if norm(e(1:3)) <= angularError && norm(e(4:6)) <= linearError
            result.Status = 1;
            break;
        end
        J = J_space(robotRBT, configuration_a, lastJointIndex);
        manipulability = sqrt(det(J * J'));
        deltaTheta = pinv(J) * e + scale * pinv(J) * (1 / manipulability);
        for j = 1:lastJointIndex
            configuration_a(j).JointPosition = configuration_a(j).JointPosition + deltaTheta(j);
        end
        e = calculateError(...
            FK_space(robotRBT, configuration_a, lastJointIndex).T, ...
            FK_space(robotRBT, configuration_b, lastJointIndex).T);
        i = i + 1;
    end
    result.Configuration = configuration_a;
    result.IterationCount = i;
    if ~exist('showVisualization', 'var')
        return;
    end
    % show visualization
    figure();
    hold on;
    % show robot
    if i > 50
        i = 50;
    end
    for j = 1:i
        show(robotRBT, configurations{j}, "Frames", "off");
    end
    set(findall(gca, 'Type', 'patch'), 'FaceAlpha', 0.1); % make transparent
    show(robotRBT, configuration_b);
    axis equal; % makes axes proportional
    view(3); % force 3D view (fixes "stretching" issues?)
    camlight; % make shaded
    axis vis3d; % keeps it from zooming in and out to fit in plot window
    grid on; % background grid
    hold off;
end
