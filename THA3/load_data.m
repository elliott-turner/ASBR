% load data from data files in dir for specified name
function data_struct = load_data(dir,name)
    % name-calbody.txt
    [header, data] = read_file(dir, strcat(name, '-calbody.txt'));
    % read lengths
    N_D = str2double(header{1});
    N_A = str2double(header{2});
    N_C = str2double(header{3});
    % copy data to matrices
    d = data(1:N_D,1:3);
    a = data(N_D+1:N_D+N_A,1:3);
    c = data(N_D+N_A+1:N_D+N_A+N_C,1:3);
    % add to struct
    data_struct.d = d;
    data_struct.a = a;
    data_struct.c = c;

    % name-calreadings.txt
    [header, data] = read_file(dir, strcat(name, '-calreadings.txt'));
    % read lengths
    N_D = str2double(header{1});
    N_A = str2double(header{2});
    N_C = str2double(header{3});
    N_frames = str2double(header{4});
    frameSize = N_D + N_A + N_C;
    % copy frame data into cell arrays of matrices
    D = {N_frames,1};
    A = {N_frames,1};
    C = {N_frames,1};
    for iFrame = 1:N_frames
        D{iFrame} = data((iFrame-1)*frameSize+1:(iFrame-1)*frameSize+N_D,:);
        A{iFrame} = data((iFrame-1)*frameSize+N_D+1:(iFrame-1)*frameSize+N_D+N_A,:);
        C{iFrame} = data((iFrame-1)*frameSize+N_D+N_A+1:(iFrame-1)*frameSize+N_D+N_A+N_C,:);
    end
    % add to struct
    data_struct.D = D;
    data_struct.A = A;
    data_struct.C = C;

    % name-empivot.txt
    [header, data] = read_file(dir, strcat(name, '-empivot.txt'));
    % read lengths
    N_G = str2double(header{1});
    N_frames = str2double(header{2});
    % copy frame data into cell arrays of matrices
    G = {N_frames,1};
    for iFrame = 1:N_frames
        G{iFrame} = data((iFrame-1)*N_G+1:(iFrame-1)*N_G+N_G,:);
    end
    % add to struct
    data_struct.G = G;

    % name-optpivot.txt
    [header, data] = read_file(dir, strcat(name, '-optpivot.txt'));
    % read lengths
    N_D_opt = str2double(header{1});
    N_H = str2double(header{2});
    N_frames = str2double(header{3});
    % copy frame data into cell arrays of matrices
    D_opt = {N_frames,1};
    H = {N_frames,1};
    frameSize = N_D_opt + N_H;
    for iFrame = 1:N_frames
        D_opt{iFrame} = data((iFrame-1)*frameSize+1:(iFrame-1)*frameSize+N_D_opt,:);
        H{iFrame} = data((iFrame-1)*frameSize+N_D_opt+1:(iFrame-1)*frameSize+N_D_opt+N_H,:);
    end
    data_struct.D_opt = D_opt;
    data_struct.H = H;
end
