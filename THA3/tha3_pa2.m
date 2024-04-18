% THA3 PA2

% 1. write function to solve for unknown transformation X
% a. quaternion analytical approach
% see solve_quaternion_transform.m

% 2. use with noisy data
% for this analysis, the calculations were performed multiple times with random
% noise and data point selection to hopefully produce more representative data
numSamples = 100;

% a. compare results using noise free and noisy data
X_clean = zeros(4,4,numSamples);
X_noisy = zeros(4,4,numSamples);
for i = 1:numSamples
    % noise free
    [q_r,q_c,t_r,t_c] = data_quaternion();
    indices = 1:length(q_r);
    X_clean(:,:,i) = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices);
    % noisy
    [q_r,q_c,t_r,t_c] = data_quaternion_noisy();
    X_noisy(:,:,i) = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices);
end
[S_clean,M_clean] = std(X_clean,0,[3]);
[S_noisy,M_noisy] = std(X_noisy,0,[3]);

% b. compare results using all data and half of the data
X_clean_half = zeros(4,4,numSamples);
X_noisy_half = zeros(4,4,numSamples);
for i = 1:numSamples
    % noise free
    [q_r,q_c,t_r,t_c] = data_quaternion();
    indices = randperm(length(q_r),round(length(q_r)/2));
    X_clean_half(:,:,i) = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices);
    % noisy
    [q_r,q_c,t_r,t_c] = data_quaternion_noisy();
    X_noisy_half(:,:,i) = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices);
end
[S_clean_half,M_clean_half] = std(X_clean_half,0,[3]);
[S_noisy_half,M_noisy_half] = std(X_noisy_half,0,[3]);
