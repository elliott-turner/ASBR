% make animations for presentation

setup = test_setup;

% animation length [seconds]
animation_length = 5;

% 2 configurations for testing
config_1a = thetaToConfiguration([0.5, 0.5, 0.5, 0.8, 0.8, 1.0, 1.0], setup.Robot);
config_1b = thetaToConfiguration([1.0, 1.0, 1.0, 1.0, 1.0, 1.5, 1.5], setup.Robot);
config_2a = thetaToConfiguration([1.2, 0.5, 0.4, 1.6, 0.0, 0.4, 0.2], setup.Robot);
config_2b = thetaToConfiguration([-1.2, 0.5, 0.4, 1.2, 0.0, 0.6, 0.2], setup.Robot);

test_config1 = thetaToConfiguration([1.2, 0.5, 0.4, 1.6, 0, 0.4, 0.2], setup.Robot);
test_config2 = thetaToConfiguration([-1.2, 0.5, 0.4, 1.2, 0, 0.6, 0.2], setup.Robot);
test_config3 = thetaToConfiguration([-2.32091309006656, 1.35112354159160, -0.448466158499428, -2.91638779175440, -2.76316625538014, 0.974815122731441, 1.00736548019515], setup.Robot);
test_config4 = thetaToConfiguration([0, 0, 0, 0, 0, 2, .5], setup.Robot);
config_a = randomConfiguration(setup.Robot);
config_b = randomConfiguration(setup.Robot);

%result = J_inverse_kinematics(setup.Robot, test_config3, test_config2, setup.LastJointIndex, 0.01, 0.001, 50)
result = J_transpose_kinematics(setup.Robot, test_config3, test_config2, setup.LastJointIndex, 0.01, 500, 0.5)

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


result = J_inverse_kinematics(setup.Robot, test_config3, test_config2, setup.LastJointIndex, 0.01, 0.001, 50);
animate(setup.Robot, setup.LastJointIndex, result.Configurations, "animations/4_a.mp4");
result = redundancy_resolution(setup.Robot, test_config3, test_config2, setup.LastJointIndex, 0.01, 0.001, 50, 5)
animate(setup.Robot, setup.LastJointIndex, result.Configurations, "animations/4_b.mp4");
