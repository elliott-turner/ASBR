% test and run tha3_pa1 code
dir = 'HW3-PA1';
outputDir = 'HW3-PA1-OUTPUT';

debug_letters = ['a','b','c','d','e','f','g'];
unknown_letters = ['h','i','j','k'];

% test again debug outputs
for iDebug = 1:length(debug_letters)
    name = strcat('pa1-debug-',debug_letters(iDebug));
    output = tha3_pa1(dir,name);
    [~,correct_output] = read_file(dir, strcat(name,'-output1.txt'));
    max(output.data - correct_output)
    write_file(outputDir,name,output);
end

% write unknown outputs
for iUnknown = 1:length(unknown_letters)
    name = strcat('pa1-unknown-',unknown_letters(iUnknown));
    output = tha3_pa1(dir,name);
    write_file(outputDir,name,output);
end

function write_file(dir,name,output)
    outputFilename = strcat(name,'-output1.txt');
    fid = fopen(fullfile(dir, outputFilename),'w');
    fprintf(fid,'%d,%d,%s\n',[output.N_C,output.N_frames,outputFilename]);
    for i = 1:size(output.data)
        fprintf(fid,'%0.2f,%0.2f,%0.2f\n',output.data(i,:));
    end
    fclose(fid);
end
