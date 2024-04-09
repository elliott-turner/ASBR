% make animations for presentation

setup = test_setup;

% animation length [seconds]
animation_length = 5;

% configurations
config_1a = thetaToConfiguration([0.5, 0.5, 0.5, 0.8, 0.8, 1.0, 1.0], setup.Robot);
config_1b = thetaToConfiguration([1.0, 1.0, 1.0, 1.0, 1.0, 1.5, 1.5], setup.Robot);
config_2a = thetaToConfiguration([1.2, 0.5, 0.4, 1.6, 0.0, 0.4, 0.2], setup.Robot);
config_2b = thetaToConfiguration([-1.2, 0.5, 0.4, 1.2, 0.0, 0.6, 0.2], setup.Robot);

result = J_inverse_kinematics(setup.Robot, config_1a, config_1b, setup.LastJointIndex, 0.01, 0.001, 50)
animate(setup.Robot, setup.LastJointIndex, result.Configurations, 'animations/h1.mp4', 'Pseudoinverse 1', result.IterationCount / animation_length);
result = J_inverse_kinematics(setup.Robot, config_2a, config_2b, setup.LastJointIndex, 0.01, 0.001, 50)
animate(setup.Robot, setup.LastJointIndex, result.Configurations, 'animations/h2.mp4', 'Pseudoinverse 2', result.IterationCount / animation_length);

result = J_transpose_kinematics(setup.Robot, config_1a, config_1b, setup.LastJointIndex, 0.01, 500, 0.5)
animate(setup.Robot, setup.LastJointIndex, result.Configurations, 'animations/i1.mp4', 'Transpose 1', result.IterationCount / animation_length);
result = J_transpose_kinematics(setup.Robot, config_2a, config_2b, setup.LastJointIndex, 0.01, 500, 0.5)
animate(setup.Robot, setup.LastJointIndex, result.Configurations, 'animations/i2.mp4', 'Transpose 2', result.IterationCount / animation_length);

result = redundancy_resolution(setup.Robot, config_1a, config_1b, setup.LastJointIndex, 0.01, 0.001, 50, 5);
animate(setup.Robot, setup.LastJointIndex, result.Configurations, 'animations/j1.mp4', 'Redundancy 1', result.IterationCount / animation_length);
result = redundancy_resolution(setup.Robot, config_2a, config_2b, setup.LastJointIndex, 0.01, 0.001, 50, 5);
animate(setup.Robot, setup.LastJointIndex, result.Configurations, 'animations/j2.mp4', 'Redundancy 2', result.IterationCount / animation_length);
