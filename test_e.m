% test part e
setup = test_setup;

config = configZero(setup.Robot, setup.LastJointIndex);
J_space(setup.Robot, config, setup.LastJointIndex)
J_body(setup.Robot, config, setup.LastJointIndex)
