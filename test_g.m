% test part g
setup = test_setup;

config = randomConfiguration(setup.Robot);
homeConfig = configZero(setup.Robot, setup.LastJointIndex);

% a
ellipsoid_plot_angular(setup.Robot, config, setup.LastJointIndex, true);
ellipsoid_plot_linear(setup.Robot, config, setup.LastJointIndex, true);

ellipsoid_plot_angular(setup.Robot, homeConfig, setup.LastJointIndex, true);
ellipsoid_plot_linear(setup.Robot, homeConfig, setup.LastJointIndex, true);

% b
mu_1 = J_isotropy(setup.Robot, config, setup.LastJointIndex);
mu_1_angular = mu_1.Angular
mu_1_linear = mu_1.Linear
mu_2 = J_condition(setup.Robot, config, setup.LastJointIndex);
mu_2_angular = mu_2.Angular
mu_2_linear = mu_2.Linear
mu_3 = J_ellipsoid_volume(setup.Robot, config, setup.LastJointIndex);
mu_3_angular = mu_3.Angular
mu_3_linear = mu_3.Linear

mu_1 = J_isotropy(setup.Robot, homeConfig, setup.LastJointIndex);
mu_1_angular = mu_1.Angular
mu_1_linear = mu_1.Linear
mu_2 = J_condition(setup.Robot, homeConfig, setup.LastJointIndex);
mu_2_angular = mu_2.Angular
mu_2_linear = mu_2.Linear
mu_3 = J_ellipsoid_volume(setup.Robot, homeConfig, setup.LastJointIndex);
mu_3_angular = mu_3.Angular
mu_3_linear = mu_3.Linear
