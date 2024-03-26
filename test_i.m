% test part i
setup = test_setup;

config_a = randomConfiguration(setup.Robot);
config_b = randomConfiguration(setup.Robot);

J_transpose_kinematics(setup.Robot, config_a, config_b, setup.LastJointIndex, 0.01, 50, true)
