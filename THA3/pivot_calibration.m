% perform pivot calibration
function [b_tip,b_post] = pivot_calibration(data,opt)
    % a. use the first frame to define local probe coordinate system
    G_0 = sum(data.G{1},1)/length(data.G{1});
    g = data.G{1} - G_0;

    % b. compute F_G for each frame
    F_D = eye(4);
    if exist('opt','var')
        data.G = data.H;
    end
    A = zeros(length(data.G)*3,6);
    b = zeros(length(data.G)*3,1);
    for iFrame = 1:length(data.G)
        if exist('opt','var')
            F_D = inv(registration(data.d,data.D_opt{iFrame}));
        end
        F_G = F_D * registration(g,data.G{iFrame});
        A((iFrame-1)*3+1:(iFrame-1)*3+3,:) = [F_G(1:3,1:3), -eye(3)]; % [R_k -I]
        b((iFrame-1)*3+1:(iFrame-1)*3+3,:) = -F_G(1:3,4); % [-p_k]
    end

    % c. solve the system
    x = A\b;
    b_tip = x(1:3,1);
    b_post = x(4:6,1);
end
