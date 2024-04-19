% THA3 PA2

% 1. write function to solve for unknown transformation X
% a. quaternion analytical approach
% see solve_quaternion_transform.m

% 2. use with noisy data
[q_r,q_c,t_r,t_c] = data_quaternion();
indices = 1:length(q_r);
X_clean = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices)
[q_r,q_c,t_r,t_c] = data_quaternion_noisy();
X_noisy = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices)


% b. compare results using all data and half of the data
[q_r,q_c,t_r,t_c] = data_quaternion();
indices = 1:round(length(q_r)/2);
X_clean_half = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices)
[q_r,q_c,t_r,t_c] = data_quaternion_noisy();
X_noisy_half = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices)


% for this analysis, the calculations were performed multiple times with random
% noise to observe noise influence
numSamples = 100;
outputDir = 'hw3_pa2_output';

% comparison of noise free and noisy data for all data points
X_clean = zeros(4,4,numSamples);
X_noisy = zeros(4,4,numSamples);
for i = 1:numSamples
    [q_r,q_c,t_r,t_c] = data_quaternion();
    indices = 1:length(q_r);
    X_clean(:,:,i) = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices);
    [q_r,q_c,t_r,t_c] = data_quaternion_rand();
    X_noisy(:,:,i) = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices);
end

% comparison of noise free and noisy data for half of data points
X_clean_half = zeros(4,4,numSamples);
X_noisy_half = zeros(4,4,numSamples);
for i = 1:numSamples
    [q_r,q_c,t_r,t_c] = data_quaternion();
    % indices = randperm(length(q_r),round(length(q_r)/2)); % also adds variance to clean
    indices = 1:round(length(q_r)/2);
    X_clean_half(:,:,i) = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices);
    [q_r,q_c,t_r,t_c] = data_quaternion_rand();
    X_noisy_half(:,:,i) = solve_quaternion_transform(q_r,q_c,t_r,t_c,indices);
end

% visualize data
visualize_transform_statistics(X_clean,'All Noise Free Samples',outputDir,'x_clean.png');
visualize_transform_statistics(X_noisy,'All Noisy Samples',outputDir,'x_noisy.png');
visualize_transform_statistics(X_clean_half,'Half of Noise Free Samples',outputDir,'x_clean_half.png');
visualize_transform_statistics(X_noisy_half,'Half of Noisy Samples',outputDir,'x_noisy_half.png');

% visualize mean and stdevof transformation matrices
function visualize_transform_statistics(T,name,dir,filename)
    T = T(1:3,:,:);
    [S,M] = std(T,0,[3]);
    scaling_factor = 5e3;
    fig = figure('visible','off');
    gca = axes;
    hold on;
    [x,y] = meshgrid(1:size(T,2), 1:size(T,1));
    for i = 1:numel(M)
        plot3(x(i),y(i),M(i),'o','MarkerFaceColor','black','MarkerEdgeColor','black');
        line([x(i),x(i)],[y(i),y(i)],...
            [M(i) - scaling_factor * S(i), M(i) + scaling_factor * S(i)],...
            'Color','#888888','LineWidth',2);
    end
    title(sprintf('%s (n=%d, stdev\\_scale=%d:1)',name,size(T,3),scaling_factor));
    xlabel('column index');
    ylabel('row index');
    zlabel('value');
    legend('mean', 'stdev');
    view(3);
    grid on;
    set(gca,'YDir','reverse');
    xlim([0.75,4.25])
    ylim([0.75,3.25])
    annotation('textbox',[0.1,0.05,0.8,0.03],'String',sprintf('mean(stdev)=%f',mean(S,'all')),'HorizontalAlignment','left','EdgeColor','none')
    hold off;
    saveas(fig,fullfile(dir,filename));
end
