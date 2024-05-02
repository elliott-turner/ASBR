setup = test_setup_pa4();

config = homeConfiguration(setup.Robot);
p_goal = [0.5,0.5,0.5];

result = IK_pa4(setup.Robot,setup.LastJointIndex,config,p_goal,0.003,100,1,0);
visualizeResult(setup,result,p_goal,"p_{goal}=[0.5,0.5,0.5], zeta=1.0, eta=0.0");
plotMeasures(setup,result);

result = IK_pa4(setup.Robot,setup.LastJointIndex,config_1,p_goal,0.003,100,0.90,0.10);
visualizeResult(setup,result,p_goal,"p_{goal}=[0.5,0.5,0.5], zeta=0.9, eta=0.1");
plotMeasures(setup,result);

function visualizeResult(setup,result,p_goal,figureTitle)
    figure();
    hold on;
    for i = 1:result.IterationCount-1
        show(setup.Robot,result.Configurations{i},"Frames","off");
    end
    set(findall(gca, 'Type', 'patch'), 'FaceAlpha', 0.1); % make transparent
    show(setup.Robot,result.Configurations{result.IterationCount},"Frames","off");
    scatter3(p_goal(1),p_goal(2),p_goal(3));
    axis equal; % makes axes proportional
    view(3); % force 3D view (fixes "stretching" issues?)
    camlight; % make shaded
    axis vis3d; % keeps it from zooming in and out to fit in plot window
    grid on; % background grid
    title(figureTitle);
    xlabel("X [m]");
    ylabel("Y [m]");
    zlabel("Z [m]");
    hold off;
end

function plotMeasures(setup,result)
    mu_angular = zeros(1,result.IterationCount);
    mu_linear = zeros(1,result.IterationCount);
    for i = 1:result.IterationCount
        mu = J_isotropy(setup.Robot,result.Configurations{i},setup.LastJointIndex-1);
        mu_angular(i) = mu.Angular;
        mu_linear(i) = mu.Linear;
    end
    errors = zeros(1,result.IterationCount);
    for i = 1:result.IterationCount
        errors(i) = norm(result.Errors{i});
    end
    figure();
    subplot(2,1,1);
    hold on;
    plot(mu_angular);
    plot(mu_linear);
    title("mu_1");
    xlabel("iteration");
    ylabel("manipulability");
    legend(["angular","linear"]);
    ylim([0,20]);
    hold off;
    subplot(2,1,2);
    hold on;
    plot(errors);
    title("norm(error)");
    xlabel("iteration");
    ylabel("norm(error) [m]");
    hold off;
end
