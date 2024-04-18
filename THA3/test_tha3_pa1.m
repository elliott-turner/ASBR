% test and run tha3_pa1 code
dir = 'HW3-PA1';
outputDir = 'HW3-PA1-OUTPUT';

debug_letters = ['a','b','c','d','e','f','g'];
unknown_letters = ['h','i','j','k'];

% test again debug outputs
for iDebug = 1:length(debug_letters)
    output = tha3_pa1(dir,strcat('pa1-debug-',debug_letters(iDebug)));
    [~,correct_output] = read_file(dir, strcat('pa1-debug-',debug_letters(iDebug),'-output1.txt'));
    max(output.data - correct_output)
end

% write unknown outputs
for iUnknown = 1:length(unknown_letters)
    output = tha3_pa1(dir,strcat('pa1-unknown-',unknown_letters(iUnknown)));
    outputFilename = strcat('pa1-unknown-',unknown_letters(iUnknown),'-output1.txt');
    fid = fopen(fullfile(outputDir, outputFilename),'w');
    fprintf(fid,'%d,%d,%s\n',[output.N_C,output.N_frames,outputFilename]);
    for i = 1:size(output.data)
        fprintf(fid,'%0.2f,%0.2f,%0.2f\n',output.data(i,:));
    end
    fclose(fid);
end
