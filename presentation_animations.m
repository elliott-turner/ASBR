% make animations for presentation

setup = test_setup;

% animation length [seconds]
animation_length = 5;

% 2 configurations for testing
config_1a = thetaToConfiguration([0.5, 0.5, 0.5, 0.8, 0.8, 1.0, 1.0], setup.Robot);
config_1b = thetaToConfiguration([1.0, 1.0, 1.0, 1.0, 1.0, 1.5, 1.5], setup.Robot);
config_2a = thetaToConfiguration([1.2, 0.5, 0.4, 1.6, 0.0, 0.4, 0.2], setup.Robot);
config_2b = thetaToConfiguration([-1.2, 0.5, 0.4, 1.2, 0.0, 0.6, 0.2], setup.Robot);

% run algorithms on configurations
result_h1 = J_inverse_kinematics(setup.Robot, config_1a, config_1b, setup.LastJointIndex, 0.01, 0.001, 50)
result_h2 = J_inverse_kinematics(setup.Robot, config_2a, config_2b, setup.LastJointIndex, 0.01, 0.001, 50)
result_i1 = J_transpose_kinematics(setup.Robot, config_1a, config_1b, setup.LastJointIndex, 0.01, 500, 0.5)
result_i2 = J_transpose_kinematics(setup.Robot, config_2a, config_2b, setup.LastJointIndex, 0.01, 500, 0.5)
result_j1 = redundancy_resolution(setup.Robot, config_1a, config_1b, setup.LastJointIndex, 0.01, 0.001, 50, 5)
result_j2 = redundancy_resolution(setup.Robot, config_2a, config_2b, setup.LastJointIndex, 0.01, 0.001, 50, 5)

% render animations of algorithm runs
mus_h1 = animate(setup.Robot, setup.LastJointIndex, result_h1.Configurations, 'animations/h1.mp4', 'Pseudoinverse 1', result_h1.IterationCount / animation_length);
mus_h2 = animate(setup.Robot, setup.LastJointIndex, result_h2.Configurations, 'animations/h2.mp4', 'Pseudoinverse 2', result_h2.IterationCount / animation_length);
mus_i1 = animate(setup.Robot, setup.LastJointIndex, result_i1.Configurations, 'animations/i1.mp4', 'Transpose 1', result_i1.IterationCount / animation_length);
mus_i2 = animate(setup.Robot, setup.LastJointIndex, result_i2.Configurations, 'animations/i2.mp4', 'Transpose 2', result_i2.IterationCount / animation_length);
mus_j1 = animate(setup.Robot, setup.LastJointIndex, result_j1.Configurations, 'animations/j1.mp4', 'Redundancy 1', result_j1.IterationCount / animation_length);
mus_j2 = animate(setup.Robot, setup.LastJointIndex, result_j2.Configurations, 'animations/j2.mp4', 'Redundancy 2', result_j2.IterationCount / animation_length);

% compare errors
fig = figure;
plot(result_h2.AngularErrors);
hold on;
plot(result_h2.LinearErrors);
plot(result_i2.AngularErrors);
plot(result_i2.LinearErrors);
plot(result_j2.AngularErrors);
plot(result_j2.LinearErrors);
xlabel('iteration')
ylabel('norm(error)')
title('Normalized Error vs Iteration for Various IK Algorithms')
legend('pinv w/o rr angular', 'pinv w/o rr linear', 'transpose angular', 'transpose linear', 'pinv w/ rr angular', 'pinv w/ rr linear');
xlim([1, 25]);
hold off;
saveas(fig, 'animations/error_comp.png')


% compare manipulability measures
fig = figure;
m_h = mus_h2.mu1;
m_j = mus_j2.mu1;
plot(m_h(:,1));
hold on;
plot(m_h(:,2));
plot(m_j(:,1));
plot(m_j(:,2));
xlabel('iteration');
ylabel('isotropy');
title('Isotropy vs Iteration With and Without Redundancy Resolution');
legend('angular w/o', 'linear w/o', 'angular w/', 'linear w/');
hold off;
saveas(fig, 'animations/mu_comp.png')
