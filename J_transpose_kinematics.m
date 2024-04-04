% uses iterative numerical transpose kinematics algorithm to
% move robot from configuration_a to configuration_b
% joints beyond lastJointIndex are ignored
function result = J_transpose_kinematics(robotRBT, configuration_a, configuration_b, lastJointIndex, error, maxIterations, scale, showVisualization)
    i = 1;
    configurations = {};
    result.Status = 0;
    e = calculateError(...
        FK_space(robotRBT, configuration_a, lastJointIndex).T, ...
        FK_space(robotRBT, configuration_b, lastJointIndex).T);
    while i <= maxIterations
        configurations{i} = configuration_a;
        if norm(e) <= error
            result.Status = 1;
            break;
        end
        deltaTheta = J_space(robotRBT, configuration_a, lastJointIndex).' * (scale .* eye(6)) * e;
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
    result.Configurations = configurations;
    if ~exist('showVisualization', 'var')
        return;
    end
    % show visualization
    figure();
    hold on;
    % show robot
    if i > maxIterations
        i = maxIterations;
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
