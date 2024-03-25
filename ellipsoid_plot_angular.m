% show the manipulability ellipsoid of robotRBT for given configuration
% joints beyuond lastJointIndex are ignored
function ellipsoid_plot_angular(robotRBT, configuration, lastJointIndex)
    J = J_space(robotRBT, configuration, lastJointIndex);
    J_omega = J(1:3, 1:end);
    A = J_omega * J_omega.';
    [v, d] = eig(A);
    T = getTransform(robotRBT, configuration, robotRBT.BodyNames{lastJointIndex});
    p = T(1:3, end);

    scaleFactor = 0.1;
    v1 = v(1:3, 1) .* (d(1, 1) * scaleFactor);
    v2 = v(1:3, 2) .* (d(2, 2) * scaleFactor);
    v3 = v(1:3, 3) .* (d(3, 3) * scaleFactor);

    [x, y, z] = ellipsoid(0, 0, 0, 1, 1, 1);
    P = p(:) + v1(:)*x(:)' + v2(:)*y(:)' + v3(:)*z(:)';

    X = reshape(P(1,:), size(x));
    Y = reshape(P(2,:), size(y));
    Z = reshape(P(3,:), size(z));

    % plot robot and ellipsoid
    figure();
    hold on;
    show(robotRBT, configuration);
    surf(X, Y, Z, 'FaceColor', 'yellow', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    fixVisualization();
    hold off;
end
