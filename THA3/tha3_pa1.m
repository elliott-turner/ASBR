% THA3 PA1
function output = tha3_pa1(dir,name)
    % load data
    data = load_data(dir,name);

    % 1. develop a 3d set registration algorithm
    % see registration.m


    % 2. develop a "pivot" calibration method
    % see pivot_calibration.m


    % 3. given a distortion calibration data set, compute the expected values
    C_expected = {length(data.D),1};
    for iFrame = 1:length(data.D)
        % a. compute transformations between trackers
        F_D = registration(data.d,data.D{iFrame});

        % b. computer transformation between calibration object and optical tracker
        F_A = registration(data.a,data.A{iFrame});

        % c. given F_D and F_a compute C_expected
        C_expected{iFrame} = zeros(length(data.c),4);
        for ic = 1:length(data.c)
            C_expected{iFrame}(ic,:) = inv(F_D) * F_A * [data.c(ic,:), 1].';
        end
    end

    % d. output C_expected
    % see below

    % 4. apply em tracking data to perform pivot calibration
    [~,b_post_em] = pivot_calibration(data);

    % see pivot_calibration.m for substeps

    % 5. apply opt tracking data to perform pivot calibration
    [~,b_post_opt] = pivot_calibration(data, true);

    % combine data for output
    output_data = zeros(2+length(C_expected)*length(data.c),3);
    output_data(1,:) = b_post_em;
    output_data(2,:) = b_post_opt;
    for iFrame = 1:length(C_expected)
        output_data(2+(iFrame-1)*length(data.c)+1:2+(iFrame-1)*length(data.c)+length(data.c),:) = C_expected{iFrame}(:,1:3);
    end
    output.data = output_data;
    output.N_C = length(data.c);
    output.N_frames = length(C_expected);
end
