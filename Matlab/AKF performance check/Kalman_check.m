% Asynchronous Kalman performance check.
clear variables
close all
clc
set(0, 'DefaultLineLineWidth', 0.8);

% Colors
blue = [0.3010, 0.7450, 0.9330];
red = [0.6350, 0.0780, 0.1840];
yellow = [0.9290, 0.6940, 0.1250];
black = [0, 0, 0];

% Settings
figures = true;

%% Load data
data = load("data/vhat_data_check_xy");
dataz  = load("data/vhat_data_check_z");

% preprocessing
data.meas_time = data.meas_time(2:end);
data.meas_pos_x = data.meas_pos_x(2:end);
data.meas_pos_y = data.meas_pos_y(2:end);
data.meas_pos_z = data.meas_pos_z(2:end);

dataz.meas_time = dataz.meas_time(2:end);
dataz.meas_pos_x = dataz.meas_pos_x(2:end);
dataz.meas_pos_y = dataz.meas_pos_y(2:end);
dataz.meas_pos_z = dataz.meas_pos_z(2:end);



%% Numerically differentiate position to get velocity estimation
vel_FD_x = (data.meas_pos_x(2:end) - data.meas_pos_x(1:end - 1))*50;
vel_FD_y = (data.meas_pos_y(2:end) - data.meas_pos_y(1:end - 1))*50;
vel_FD_z = (dataz.meas_pos_z(2:end) - dataz.meas_pos_z(1:end - 1))*50;

vel_FD_x_accurate = gradient(data.meas_pos_x)./gradient(data.meas_time); 
vel_FD_x_accurate = vel_FD_x_accurate(2:end);

data.est_vel_x = data.est_vel_x(1:end - 1);
data.est_vel_y = data.est_vel_y(1:end - 1);
dataz.est_vel_z = dataz.est_vel_z(1:end - 1);

% Ts = 0.02;
% N_meas = length(data.meas_time);
% t_meas = (0:N_meas-1)'*Ts;
% Ts = 0.01;
% N_est = length(data.est_time);
% t_est = (0:N_est-1)'*Ts;

t_meas = data.meas_time - data.est_time(1);
t_est  = data.est_time - data.est_time(1) + 0.01;

t_measz = dataz.meas_time - dataz.est_time(1);
t_estz  = dataz.est_time - dataz.est_time(1) + 0.01;

%% Plots to compare the velocity estimation of the kalman to the numerically differentiated velocity

% ========
% Position
% ========
% - only x
if figures
    figure('Name','x AKF')
    hold on
    plot(t_meas, data.meas_pos_x, 'o', 'Color', blue)
    plot(t_est, data.est_pos_x, '*', 'Color', red)
    legend('Measured position', 'Kalman estimated position')
    xlabel('Time (s)')
    axis tight
    ylabel('x-position (m)')

    % - all directions
    figure('Name','Comparison measured pos with kalman est pos')

    subplot(311)
    hold on
    plot(t_meas, data.meas_pos_x, 'Color', blue)
    plot(t_est, data.est_pos_x, 'Color', red)
    legend('Measured position', 'Kalman estimated position')
    xlabel('Time [s]')
    axis tight
    ylabel('X Position [m]')

    subplot(312)
    hold on
    plot(t_meas, data.meas_pos_y, 'Color', blue)
    plot(t_est, data.est_pos_y, 'Color', red)
    legend('Measured position', 'Kalman estimated position')
    xlabel('Time [s]')
    axis tight
    ylabel('Y Position [m]')

    subplot(313)
    hold on
    plot(t_measz, dataz.meas_pos_z, 'Color', blue)
    plot(t_estz, dataz.est_pos_z, 'Color', red)
    legend('Measured position', 'Kalman estimated position')
    xlabel('Time [s]')
    axis tight
    ylabel('Z Position [m]')
end

% ==========
% Velocities
% ==========
if figures
    % only x
    figure('Name','Comparison estimated vel with num diff vel')
    hold on
    plot(t_meas(1:end-1), vel_FD_x, 'Color', blue)
    plot(t_meas(1:end-1), vel_FD_x_accurate, 'Color', yellow)
    plot(t_est(1:end-1), data.est_vel_x, 'Color', red)
    legend('Finite diff', 'Finite diff accurate', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('X Velocity [m/s]')
    
    
    % all directions
    figure('Name','Comparison estimated vel with num diff vel')

    subplot(311)
    hold on
    plot(t_meas(1:end-1), vel_FD_x, 'Color', blue)
    plot(t_est(1:end-1), data.est_vel_x, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('X Velocity [m/s]')

    subplot(312)
    hold on
    plot(t_meas(1:end-1), vel_FD_y, 'Color', blue)
    plot(t_est(1:end-1), data.est_vel_y, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('Y Velocity [m/s]')

    subplot(313)
    hold on
    plot(t_measz(1:end-1), vel_FD_z, 'Color', blue)
    plot(t_estz(1:end-1), dataz.est_vel_z, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('Z Velocity [m/s]')
end

%% Compare ZOH using only measurements VS AKF predictions

reference = interp1(t_meas,data.meas_pos_x,t_est);
zoh = interp1(t_meas,data.meas_pos_x,t_est,'previous');

figure
hold on
plot(t_meas, data.meas_pos_x,'o','Color',blue)
plot(t_est,reference,'--','Color',black)
plot(t_est,zoh,'-x','Color',yellow)
plot(t_est, data.est_pos_x, '-x', 'Color',red)
legend('measurements','reference','zoh','AKF')








