% show angular manipulability ellipsoid of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function [p, A] = ellipsoid_plot_angular(robotRBT, configuration, lastJointIndex, showVisualization)
    J = J_space(robotRBT, configuration, lastJointIndex);
    J_omega = J(1:3, 1:end);
    A = J_omega * J_omega.';
    T = getTransform(robotRBT, configuration, robotRBT.BodyNames{lastJointIndex});
    p = T(1:3, end);

    if ~exist('showVisualization', 'var')
        return;
    end

    % plot robot and ellipsoid
    figure();
    hold on;
    show(robotRBT, configuration);
    showEllipsoid(p, A, 0.1, 'yellow');
    fixVisualization();
    hold off;
end
