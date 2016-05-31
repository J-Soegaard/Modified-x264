clear all
close all
clc

txt = fileread('100frames_EPZS_print.log');

pattern = 'SP:*[^\n]*';
data = regexp(txt, pattern,'match');

SP = 0;
for k = 1:length(data)
    MB = strread(data{k}(4:end));
    SP = SP + sum(MB);
end

disp(['EPZS MSP per frame: ' num2str(SP/100e6,4)])

%% New sets

txt = fileread('100frames_EPZS_newSets_print.log');

pattern = 'SP:*[^\n]*';
data = regexp(txt, pattern,'match');

SP = 0;
for k = 1:length(data)
    MB = strread(data{k}(4:end));
    SP = SP + sum(MB);
end

disp(['EPZS New Sets MSP per frame: ' num2str(SP/100e6,4)])
