% Analyse computation times MPC for varying obstacle types and number of
% obstacles.

clear variables
close all
clc
set(0, 'DefaultLineLineWidth', 1.5);

figures = true;

% Colors
blue = [0.3010, 0.7450, 0.9330];
red = [0.6350, 0.0780, 0.1840];
yellow = [0.9290, 0.6940, 0.1250];
set(0, 'DefaultLineLineWidth', 1.5);


%% Load data
files = dir('data/*.mat');

% Plot computation times and calculate average and 
for file = files'
    data = load(file.name);
    average = mean(data.time);
    peak    = max(data.time);
    
    if figures
        figure('Name',file.name)
        plot(data.time, 'Color', red)
        xlabel('Iteration')
        ylabel('Computation time [s]')
    end
    
    fprintf('===================================================\n')
    display(file.name)
    display(average)
    display(peak)
    
end


