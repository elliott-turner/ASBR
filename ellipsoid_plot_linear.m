% show linear manipulability ellipsoid of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function [p, A] = ellipsoid_plot_linear(robotRBT, configuration, lastJointIndex, showVisualization)
    J = J_space(robotRBT, configuration, lastJointIndex);
    J_nu = J(4:6, 1:end);
    A = J_nu * J_nu.';
    T = getTransform(robotRBT, configuration, robotRBT.BodyNames{lastJointIndex});
    p = T(1:3, end);

    if ~exist('showVisualization', 'var')
        return;
    end

    % plot robot and ellipsoid
    figure();
    hold on;
    show(robotRBT, configuration);
    showEllipsoid(p, A, 0.5, 'magenta');
    fixVisualization();
    hold off;
end
