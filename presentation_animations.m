% make animations for presentation

setup = test_setup;

config = thetaToConfiguration([0, 0.5, 1, -1.5, 0, 1, 0], setup.Robot);

% config_a = thetaToConfiguration([0, 0.5, 1, -1.5, 0, 1, 0], setup.Robot);
% config_b = thetaToConfiguration([0.3, 0.7, 1, -1.2, 0, 0.8, 0], setup.Robot);

test_config1 = thetaToConfiguration([1, 1, 1, 1, 1, pi()/2, pi()/2], setup.Robot);
test_config2 = thetaToConfiguration([0, 0, 0, 0, 0, 0, .5], setup.Robot);
config_a = randomConfiguration(setup.Robot);
config_b = randomConfiguration(setup.Robot);

result = J_inverse_kinematics(setup.Robot, config_a, test_config2, setup.LastJointIndex, 0.01, 0.001, 50)
% result = J_transpose_kinematics(setup.Robot, config_a, config_b, setup.LastJointIndex, 0.01, 50, 0.5)

animate(setup.Robot, setup.LastJointIndex, result.Configurations, "animations/test.mp4")
