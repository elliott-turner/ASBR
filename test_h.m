% test part h
setup = test_setup;

config_a = randomConfiguration(setup.Robot);
config_b = randomConfiguration(setup.Robot);

J_inverse_kinematics(setup.Robot, config_a, config_b, setup.LastJointIndex, 0.01, 0.001, 50, true)
