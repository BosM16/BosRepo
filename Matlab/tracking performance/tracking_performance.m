% Analyse computation times MPC for varying obstacle types and number of
% obstacles.

% clear variables
close all
clc
set(0, 'DefaultLineLineWidth', 1.5);

figures = true;
prints = true;

% Colors
blue = [0.3010, 0.7450, 0.9330];
red = [0.6350, 0.0780, 0.1840];
yellow = [0.9290, 0.6940, 0.1250];
set(0, 'DefaultLineLineWidth', 1.1);


%% Load data
files = dir('data/*.mat');

% Plot computation times and calculate average and peak
for file = files'
    data = load(file.name);
    
    % preprocess data to have the same length
    data.drawn_path_x = data.drawn_path_x(1:length(data.real_path_x));
    data.drawn_path_y = data.drawn_path_y(1:length(data.real_path_x));
    data.drawn_path_z = data.drawn_path_z(1:length(data.real_path_x));
    
    
    t = 0:0.01:(length(data.drawn_path_z)-1)*0.01;
    
    % tracking errors
    ex = data.drawn_path_x - data.real_path_x;
    ey = data.drawn_path_y - data.real_path_y;
    ez = data.drawn_path_z - data.real_path_z;
    emag = vecnorm([ex' ey' ez']');
    
    ex_avg = mean(ex);
    ey_avg = mean(ey);
    ez_avg = mean(ez);
    emag_avg = mean(emag);
    
    ex_peak = max(abs(ex));
    ey_peak = max(abs(ey));
    ez_peak = max(abs(ez));
    emag_peak = max(abs(emag));

    
    if figures    
        
        figure('Name',file.name(1:end-4))
        
        subplot(3,2,[1 3 5])
        plot3(data.drawn_path_x, data.drawn_path_y, data.drawn_path_z,'Color', red, 'LineStyle', '--')
        hold on
        plot3(data.real_path_x, data.real_path_y, data.real_path_z,'Color', blue)
        legend('Reference trajectory', 'Tracked result')
        
        subplot(3,2,2)
        plot(t, ex, 'Color', yellow)
        title('X tracking error')
        xlabel('Time (s)')
        ylabel('Error (m)')
        
        subplot(3,2,4)
        plot(t, ey, 'Color', yellow)
        title('Y tracking error')
        xlabel('Time (s)')
        ylabel('Error (m)')
        
        subplot(3,2,6)
        plot(t, ey, 'Color', yellow)
        title('Z tracking error')
        xlabel('Time (s)')
        ylabel('Error (m)')
              
    end
    
    if prints
        fprintf('===================================================\n')
        display(file.name)
        display(emag_avg)
        display(emag_peak)
    end
end


