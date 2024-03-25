function showScrewAxes(robotRBT, configuration, lastJointIndex)
    % show screw axes
    for i = 1:lastJointIndex
        t = getTransform(robotRBT, configuration, robotRBT.BodyNames{i});
        q_i = t(1:3, end);
        w_i = t(1:3, 1:3) * robotRBT.Bodies{1,i}.Joint.JointAxis.';
        w_i = w_i .* 0.5;
        quiver3(q_i(1), q_i(2), q_i(3), w_i(1), w_i(2), w_i(3), 'LineWidth', 4, 'Color', 'yellow');
    end
end
