% test part b
setup = test_setup;

config = randomConfiguration(setup.Robot);
result = FK_space(setup.Robot, config, setup.LastJointIndex, true);
result.T
getTransform(setup.Robot, config, setup.Robot.BodyNames{setup.LastJointIndex})
