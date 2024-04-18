% read data from file
function [header, data] = read_file(dir, name)
    filedata = importdata(fullfile(dir, name),',',1);
    header = filedata.textdata;
    if length(header) == 1 % idk why this is only sometimes needed?
        header = strsplit(filedata.textdata{1}); % values in fist line
    end
    data = filedata.data; % matrix of numbers from remaining lines
end
