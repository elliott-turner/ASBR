function animate(robotRBT, lastJointIndex, configurations, fileName, plotTitle, frameRate)
    prev_mu_1 = [];
    prev_mu_2 = [];
    prev_mu_3 = [];
    fig = figure;
    x0=10;
    y0=10;
    width = 1200;
    height = 800;
    axis tight manual
    set(gcf,'position',[x0,y0,width,height])
    writer = VideoWriter(fileName, 'MPEG-4');
    writer.Quality = 100;
    writer.FrameRate = frameRate;
    open(writer);
    for i = 1:length(configurations)
        clf(fig);
        [p_angular, A_angular] = ellipsoid_plot_angular(robotRBT, configurations{i}, lastJointIndex);
        [p_linear, A_linear] = ellipsoid_plot_linear(robotRBT, configurations{i}, lastJointIndex);
        subplot(1, 2, 1);
        hold on;
        show(robotRBT, configurations{i}, "Frames", "off");
        showEllipsoid(p_angular, A_angular, 0.1, 'yellow');
        showEllipsoid(p_linear, A_linear, 0.5, 'magenta');
        fixVisualization();
        hold off;
        title('Robot Configuration');
        xlabel('x [m]')
        ylabel('y [m]')
        zlabel('z [m]')

        mu_1 = J_isotropy(robotRBT, configurations{i}, lastJointIndex);
        prev_mu_1 = [prev_mu_1; [mu_1.Angular, mu_1.Linear]];
        subplot(3, 2, 2);
        plot(1:size(prev_mu_1, 1), prev_mu_1(:, 1), '-', 1:size(prev_mu_1, 1), prev_mu_1(:, 2), '-');
        legend({'Angular', 'Linear'});
        title('\mu_1 (isotropy)');
        xlabel('iteration')

        mu_2 = J_condition(robotRBT, configurations{i}, lastJointIndex);
        prev_mu_2 = [prev_mu_2; [mu_2.Angular, mu_2.Linear]];
        subplot(3, 2, 4);
        plot(1:size(prev_mu_2, 1), prev_mu_2(:, 1), '-', 1:size(prev_mu_2, 1), prev_mu_2(:, 2), '-');
        legend({'Angular', 'Linear'});
        title('\mu_2 (condition)');
        xlabel('iteration')

        mu_3 = J_ellipsoid_volume(robotRBT, configurations{i}, lastJointIndex);
        prev_mu_3 = [prev_mu_3; [mu_3.Angular, mu_3.Linear]];
        subplot(3, 2, 6);
        plot(1:size(prev_mu_3, 1), prev_mu_3(:, 1), '-', 1:size(prev_mu_3, 1), prev_mu_3(:, 2), '-');
        legend({'Angular', 'Linear'});
        title('\mu_3 (volume)');
        xlabel('iteration')

        sgtitle(plotTitle);

        frame = getframe(fig);
        writeVideo(writer, frame);
    end
    close(writer);
    close(fig);
end
