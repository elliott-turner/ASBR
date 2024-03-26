% test part j
setup = test_setup;

config_a = randomConfiguration(setup.Robot);
config_b = randomConfiguration(setup.Robot);

redundancy_resolution(setup.Robot, config_a, config_b, setup.LastJointIndex, 0.01, 0.001, 50, 2e-4, true)
