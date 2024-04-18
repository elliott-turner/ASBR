% THA3 PA2

% 1. write function to solve for unknown transformation X
% a. quaternion analytical approach
% see solve_quaternion_transform.m

% 2. use with noisy data
% a. compare results using noise free and noisy data
% noise free
[q_r,q_c,t_r,t_c] = data_quaternion();
X_noise_free = solve_quaternion_transform(q_r,q_c,t_r,t_c,length(q_Robot_config))
% noisy
[q_r,q_c,t_r,t_c] = data_quaternion_noisy();
X_noisy = solve_quaternion_transform(q_r,q_c,t_r,t_c,length(q_Robot_config))

% b. compare results using all data and half of the data
% noise free
[q_r,q_c,t_r,t_c] = data_quaternion();
X_noise_free_half = solve_quaternion_transform(q_r,q_c,t_r,t_c,5)
% noisy
[q_r,q_c,t_r,t_c] = data_quaternion_noisy();
X_noisy_half = solve_quaternion_transform(q_r,q_c,t_r,t_c,5)
