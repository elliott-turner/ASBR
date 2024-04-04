% test part i
setup = test_setup;

% config_a = randomConfiguration(setup.Robot);
% config_b = randomConfiguration(setup.Robot);

config_a = thetaToConfiguration([0, 0.5, 1, -1.5, 0, 1, 0], setup.Robot);
config_b = thetaToConfiguration([0.3, 0.7, 1, -1.2, 0, 0.8, 0], setup.Robot);

J_transpose_kinematics(setup.Robot, config_a, config_b, setup.LastJointIndex, 0.001, 10, 0.5, true)
