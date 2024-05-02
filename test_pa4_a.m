setup = test_setup_pa4();

p_goal = [0.5,0.5,0.5];
config_a = homeConfiguration(setup.Robot);

result = IK_pa4_a(setup.Robot,setup.LastJointIndex,config_a,p_goal,0.003,100);

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
hold off;
