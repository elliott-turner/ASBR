% show linear manipulability ellipsoid of robotRBT for given configuration
% joints beyond lastJointIndex are ignored
function ellipsoid_plot_linear(robotRBT, configuration, lastJointIndex)
    J = J_space(robotRBT, configuration, lastJointIndex);
    J_nu = J(4:6, 1:end);
    A = J_nu * J_nu.';
    T = getTransform(robotRBT, configuration, robotRBT.BodyNames{lastJointIndex});
    p = T(1:3, end);

    % plot robot and ellipsoid
    figure();
    hold on;
    show(robotRBT, configuration);
    showEllipsoid(p, A, 0.5);
    fixVisualization();
    hold off;
end
