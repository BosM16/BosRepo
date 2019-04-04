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
set(0, 'DefaultLineLineWidth', 1.1);


%% Load data
files = dir('data/*.mat');

% Plot computation times and calculate average and 
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
        subplot(5,1,[1 2])
        plot3(data.drawn_path_x, data.drawn_path_y, data.drawn_path_z,'Color', red, 'LineStyle', '--')
        hold on
        plot3(data.real_path_x, data.real_path_y, data.real_path_z,'Color', blue)
        legend('Reference trajectory', 'Tracked result')
        
        
        subplot(5,1,3)
        plot(t, ex, 'Color', yellow)
        title('x tracking error')
        xlabel('time [s]')
        ylabel('x error [m]')
        
        subplot(5,1,4)
        plot(t, ey, 'Color', yellow)
        title('y tracking error')
        xlabel('time [s]')
        ylabel('y error [m]')
        
        subplot(5,1,5)
        plot(t, ey, 'Color', yellow)
        title('z tracking error')
        xlabel('time [s]')
        ylabel('z error [m]')
              
%         figure('Name',strcat(file.name(1:end-4), ' - 3d view'))
%         plot3(data.drawn_path_x, data.drawn_path_y, data.drawn_path_z)
%         hold on
%         plot3(data.real_path_x, data.real_path_y, data.real_path_z)
%         
%         figure('Name',strcat(file.name(1:end-4), ' - x tracking'))
%         subplot(211)
%         hold on
%         plot(t, data.drawn_path_x ,'Color', red)
%         plot(t, data.real_path_x, 'Color', blue)
%         xlabel('time [s]')
%         ylabel('x position [m]')
%         subplot(212)
%         plot(t, ex, 'Color', yellow)
%         xlabel('time [s]')
%         ylabel('x error [m]')
%         
%         figure('Name',strcat(file.name(1:end-4), ' - y tracking'))
%         subplot(211)
%         hold on
%         plot(t, data.drawn_path_y ,'Color', red)
%         plot(t, data.real_path_y, 'Color', blue)
%         xlabel('time [s]')
%         ylabel('z position [m]')
%         subplot(212)
%         plot(t, ey, 'Color', yellow)
%         xlabel('time [s]')
%         ylabel('y error [m]')
%         
%         figure('Name',strcat(file.name(1:end-4), ' - z tracking'))
%         subplot(211)
%         hold on
%         plot(t, data.drawn_path_z ,'Color', red)
%         plot(t, data.real_path_z, 'Color', blue)
%         xlabel('time [s]')
%         ylabel('z position [m]')
%         subplot(212)
%         plot(t, ey, 'Color', yellow)
%         xlabel('time [s]')
%         ylabel('z error [m]')
    end
    
    fprintf('===================================================\n')
    display(file.name)
    display(emag_avg)
    display(emag_peak)
    
end


