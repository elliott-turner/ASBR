function animate(robotRBT, lastJointIndex, configurations, fileName)
    prev_mu_1 = [];
    prev_mu_2 = [];
    prev_mu_3 = [];
    fig = figure;
    axis tight manual
    writer = VideoWriter(fileName);
    writer.FrameRate = 10;
    open(writer);
    for i = 1:length(configurations)
        subplot(1, 2, 1);
        show(robotRBT, configurations{i}, "Frames", "off");
        title('Robot Configuration');

        mu_1 = J_isotropy(robotRBT, configurations{i}, lastJointIndex);
        prev_mu_1 = [prev_mu_1; [mu_1.Angular, mu_1.Linear]];
        subplot(3, 2, 2);
        plot(1:size(prev_mu_1, 1), prev_mu_1(:, 1), '-o', 1:size(prev_mu_1, 1), prev_mu_1(:, 2), '-o');
        legend({'Angular', 'Linear'});
        title('mu_1');

        mu_2 = J_condition(robotRBT, configurations{i}, lastJointIndex);
        prev_mu_2 = [prev_mu_2; [mu_2.Angular, mu_2.Linear]];
        subplot(3, 2, 4);
        plot(1:size(prev_mu_2, 1), prev_mu_2(:, 1), '-o', 1:size(prev_mu_2, 1), prev_mu_2(:, 2), '-o');
        legend({'Angular', 'Linear'});
        title('mu_2');

        mu_3 = J_ellipsoid_volume(robotRBT, configurations{i}, lastJointIndex);
        prev_mu_3 = [prev_mu_3; [mu_3.Angular, mu_3.Linear]];
        subplot(3, 2, 6);
        plot(1:size(prev_mu_3, 1), prev_mu_3(:, 1), '-o', 1:size(prev_mu_3, 1), prev_mu_3(:, 2), '-o');
        legend({'Angular', 'Linear'});
        title('mu_3');

        frame = getframe(fig);
        writeVideo(writer, frame);
    end
    close(writer);
    close(fig);
end
