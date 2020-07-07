% Asynchronous Kalman performance check.
clear variables
close all
clc
set(0, 'DefaultLineLineWidth', 0.6);

% Colors
blue = [0.3010, 0.7450, 0.9330];
red = [0.6350, 0.0780, 0.1840];
yellow = [0.9290, 0.6940, 0.1250];
black = [0, 0, 0];

% Settings
all_figures = false;
figures = true;
zfig = false;


%% Load data
% data = load("data/varying_accelerations");
% data = load("data/kalman_check_1e_1");
% data = load("data/kalman_check_1e_3");
% data = load("data/kalman_check_1e_5");
% data = load("data/vhat_data_check_xy");
data = load("data/kalman_check_Qvel_1e1");

xrange_all = [0,data.est_time(end)-data.est_time(1)];
xrange_Qvel_1e1_low = [26.4,26.6];
xrange_Qvel_1e1_high = [35.5,35.85];
xrange_emin5 = [3.6,4.6];
xrange_med = [8.44,8.8];
xrange_high = [6.3,6.65];

% Select x range based on data file & part of data file you want to zoom.
xrange = xrange_Qvel_1e1_low;
% xrange = xrange_all;

% preprocessing, first value of meas time is rubbish
data.meas_time = data.meas_time(2:end);
data.meas_pos_x = data.meas_pos_x(2:end);
data.meas_pos_y = data.meas_pos_y(2:end);
data.meas_pos_z = data.meas_pos_z(2:end);


data.est_time = data.est_time(2:end);
data.est_pos_x = data.est_pos_x(2:end);
data.est_pos_y = data.est_pos_y(2:end);
data.est_pos_z = data.est_pos_z(2:end);
data.est_vel_x = data.est_vel_x(2:end);
data.est_vel_y = data.est_vel_y(2:end);
data.est_vel_z = data.est_vel_z(2:end);


%% Numerically differentiate position to get velocity estimation
dt_meas = gradient(data.meas_time);
vel_FD_x = gradient(data.meas_pos_x)./dt_meas;
vel_FD_y = gradient(data.meas_pos_y)./dt_meas;
vel_FD_z = gradient(data.meas_pos_z)./dt_meas;

% Ts = 0.02;
% N_meas = length(data.meas_time);
% t_meas = (0:N_meas-1)'*Ts;
% Ts = 0.01;
% N_est = length(data.est_time);
% t_est = (0:N_est-1)'*Ts;

t_meas = data.meas_time - data.est_time(1);
t_est  = data.est_time - data.est_time(1) + 0.01;

[~,indexl] = min(abs(t_est-xrange(1)));
[~,indexr] = min(abs(t_est-xrange(2)));


%% Plots to compare the velocity estimation of the kalman to the numerically differentiated velocity

% ========
% Position
% ========
if all_figures
% - only x
    figure('Name','x AKF')
    hold on
    plot(t_meas, data.meas_pos_x, 'o', 'Color', blue)
    plot(t_est, data.est_pos_x, '*', 'Color', red)
    legend('Measured position', 'Kalman estimated position')
    xlabel('Time (s)')
    axis tight
    ylabel('x-position (m)')
end

if all_figures
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
    plot(t_measz, data.meas_pos_z, 'Color', blue)
    plot(t_estz, data.est_pos_z, 'Color', red)
    legend('Measured position', 'Kalman estimated position')
    xlabel('Time [s]')
    axis tight
    ylabel('Z Position [m]')
end

% ==========
% Velocities
% ==========
if figures
    figure('Name','Comparison estimated vel with num diff vel')
    hold on
    plot(t_meas, vel_FD_x, '-o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
    plot(t_est, data.est_vel_x, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
%     xlim([3.35,4.6]);
    % ylim([-0.58, -0.47]);
    % xlim(xrange_high);
    % ylim([-0.2, 0.2]);
    xlim(xrange);
    % ylim([-0.2, 0.2]);
    legend('Finite diff', 'Kalman estimation')
    xlabel('Time (s)')
    % axis tight
    ylabel('Velocity (m/s)')
end

if all_figures    
    
    % all directions
    figure('Name','Comparison estimated vel with num diff vel')

    subplot(311)
    hold on
    plot(t_meas, vel_FD_x, 'Color', blue)
    plot(t_est, data.est_vel_x, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('X Velocity [m/s]')

    subplot(312)
    hold on
    plot(t_meas, vel_FD_y, 'Color', blue)
    plot(t_est, data.est_vel_y, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('Y Velocity [m/s]')

    subplot(313)
    hold on
    plot(t_measz, vel_FD_z, 'Color', blue)
    plot(t_estz, data.est_vel_z, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('Z Velocity [m/s]')
end


% =============
% Accelerations
% =============
fs = mean(1./dt_meas);
f0 = 0.4;
fc = f0; % cutoff frequency (chosen)
fcn = fc/(fs/2); % normalized cutoff frequency (as butter() accepts)
nb = 3;
[B, A] = butter(nb,fcn);
vel_x_filt = filtfilt(B,A,vel_FD_x);
acc_FD_x = gradient(vel_x_filt)./dt_meas;
acc_x_filt = filtfilt(B,A,acc_FD_x);

acc_t_est = interp1(t_meas,acc_x_filt,t_est);
accRMS = rms(acc_t_est(indexl:indexr));
display(accRMS)


if figures
        % only x
    figure('Name','Acceleration')
    hold on
    plot(t_meas, acc_FD_x, 'Color', blue)
    plot(t_meas, acc_x_filt, 'Color', red)
%     plot(t_est(1:end-1), data.est_vel_x, 'Color', red)
    legend('Finite diff','Filt')
    xlabel('Time (s)')
    axis tight
    ylabel('X Acceleration (m/s^2)')
    xlim([xrange])
end
    

if all_figures
    % only x
    figure('Name','Acceleration')
    hold on
    plot(t_meas, acc_FD_x, 'Color', blue)
    plot(t_meas, acc_x_filt, 'Color', red)
%     plot(t_est(1:end-1), data.est_vel_x, 'Color', red)
    legend('Finite diff','Filt')
    xlabel('Time (s)')
    axis tight
    ylabel('X Acceleration (m/s^2)')
end


%% Compare ZOH using only measurements VS AKF predictions
reference = interp1(t_meas,data.meas_pos_x,t_est);
reffilt = filtfilt(B,A,data.meas_pos_x);
ZOH = interp1(t_meas,data.meas_pos_x,t_est,'previous');
if figures
    % pos
    figure
    hold on
    plot(t_est, reference, '--o', 'MarkerSize',3.0,'Color',black,'markerfacecolor',black)
    plot(t_meas, data.meas_pos_x,'o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
    plot(t_est,ZOH,'-x','MarkerSize',7.0,'Color',yellow)
    plot(t_est, data.est_pos_x, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
    legend('Reference','Raw measurements','ZOH','AKF')
    xlim(xrange);
%     ylim([-0.723, -0.7]);
    xlabel('Time (s)')
    ylabel('Position (m)')
    yticks([-2.82, -2.8, -2.78])
%     yticks([-0.72, -0.71, -0.7])  % med

end

% Calculate error of ZOH / AKF wrt reference (avg and peak)
ZOH_error = abs(ZOH - reference);
AKF_error = abs(data.est_pos_x - reference);

ZOH_error_range = ZOH_error(indexl:indexr);
AKF_error_range = AKF_error(indexl:indexr);
ZOHmean = mean(ZOH_error_range);
ZOHpeak = max(ZOH_error_range);
AKFmean = mean(AKF_error_range);
AKFpeak = max(AKF_error_range);
display(ZOHmean)
display(ZOHpeak)
display(AKFmean)
display(AKFpeak)


figure
plot(ZOH_error(indexl:indexr))
hold on
plot(AKF_error(indexl:indexr))
legend('ZOH','AKF')


%% Mooie combinatieplot
figure('Name','Comparison estimated vel with num diff vel')

subplot(3,1,[1 2])
hold on
% plot(t_meas, reffilt, 'o','MarkerSize',5.0,'Color',black,'markerfacecolor',black)
plot(t_meas, data.meas_pos_x,'o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
plot(t_est,ZOH,'-x','MarkerSize',7.0,'Color',yellow)
plot(t_est, data.est_pos_x, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
legend('Added noise measurements','ZOH','AKF')
% legend('Raw measurements','Added noise measurements','ZOH','AKF')
xlim(xrange);
% ylim([-0.58, -0.47]);
% ylim([-0.723, -0.7]);
xlabel("Time (s)",'FontSize',11);
ylabel('Position (m)','FontSize',11);
set(gca,'FontSize',11)

subplot(3,1,3)
title({'';''})
hold on
plot(t_meas, vel_FD_x, '-o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
plot(t_est, data.est_vel_x, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
% xlim([3.35,4.6]);
% ylim([-0.58, -0.47]);
% xlim(xrange_high);
% ylim([-0.2, 0.2]);
xlim(xrange);
% ylim([-0.2, 0.2]);
legend('Finite Difference', 'AKF')
xlabel('Time (s)','FontSize',11)
% axis tight
ylabel('Velocity (m/s)','FontSize',11)
set(gca,'FontSize',11)


% z
if zfig
    reference = interp1(t_meas,data.meas_pos_x,t_est);
    ZOHz = interp1(t_meas,data.meas_pos_z,t_est,'previous');

    figure('Name','Z Comparison estimated vel with num diff vel')

    subplot(3,1,[1 2])
    hold on
    plot(t_meas, data.meas_pos_z,'o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
    plot(t_est,ZOHz,'-x','MarkerSize',7.0,'Color',yellow)
    plot(t_est, data.est_pos_z, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
    legend('Measurements','ZOH','AKF')
    xlim(xrange);
    % ylim([-0.58, -0.47]);
    % ylim([-0.723, -0.7]);
    xlabel("Time (s)",'FontSize',11);
    ylabel('Position (m)','FontSize',11);
    set(gca,'FontSize',11)

    subplot(3,1,3)
    title({'';''})
    hold on
    plot(t_meas, vel_FD_z, '-o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
    plot(t_est, data.est_vel_z, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
    % xlim([3.35,4.6]);
    % ylim([-0.58, -0.47]);
    % xlim(xrange_high);
    % ylim([-0.2, 0.2]);
    xlim(xrange);
    % ylim([-0.2, 0.2]);
    legend('Finite Difference', 'AKF')
    xlabel('Time (s)','FontSize',11)
    % axis tight
    ylabel('Velocity (m/s)','FontSize',11)
    set(gca,'FontSize',11)
end

 








%% ========================================================================
%  ========================================================================



%% Load data
% data = load("data/varying_accelerations");
% data = load("data/kalman_check_1e_1");
% data = load("data/kalman_check_1e_3");
% data = load("data/vhat_data_check_xy");
data = load("data/kalman_check_Qvel_1e1");

xrange_all = [0,data.est_time(end)-data.est_time(1)];
xrange_Qvel_1e1_low = [26.4,26.6];
xrange_Qvel_1e1_high = [35.5,35.85];
xrange_emin5 = [3.35,4.6];
xrange_med = [8.44,8.8];
xrange_high = [6.3,6.65];

% Select x range based on data file & part of data file you want to zoom.
xrange = xrange_Qvel_1e1_high;
% xrange = xrange_all;

% preprocessing, first value of meas time is rubbish
data.meas_time = data.meas_time(2:end);
data.meas_pos_x = data.meas_pos_x(2:end);
data.meas_pos_y = data.meas_pos_y(2:end);
data.meas_pos_z = data.meas_pos_z(2:end);


data.est_time = data.est_time(2:end);
data.est_pos_x = data.est_pos_x(2:end);
data.est_pos_y = data.est_pos_y(2:end);
data.est_pos_z = data.est_pos_z(2:end);
data.est_vel_x = data.est_vel_x(2:end);
data.est_vel_y = data.est_vel_y(2:end);
data.est_vel_z = data.est_vel_z(2:end);


%% Numerically differentiate position to get velocity estimation
dt_meas = gradient(data.meas_time);
vel_FD_x = gradient(data.meas_pos_x)./dt_meas;
vel_FD_y = gradient(data.meas_pos_y)./dt_meas;
vel_FD_z = gradient(data.meas_pos_z)./dt_meas;

% Ts = 0.02;
% N_meas = length(data.meas_time);
% t_meas = (0:N_meas-1)'*Ts;
% Ts = 0.01;
% N_est = length(data.est_time);
% t_est = (0:N_est-1)'*Ts;

t_meas = data.meas_time - data.est_time(1);
t_est  = data.est_time - data.est_time(1) + 0.01;

[~,indexl] = min(abs(t_est-xrange(1)));
[~,indexr] = min(abs(t_est-xrange(2)));


%% Plots to compare the velocity estimation of the kalman to the numerically differentiated velocity

% ========
% Position
% ========
if all_figures
% - only x
    figure('Name','x AKF')
    hold on
    plot(t_meas, data.meas_pos_x, 'o', 'Color', blue)
    plot(t_est, data.est_pos_x, '*', 'Color', red)
    legend('Measured position', 'Kalman estimated position')
    xlabel('Time (s)')
    axis tight
    ylabel('x-position (m)')
end

if all_figures
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
    plot(t_measz, data.meas_pos_z, 'Color', blue)
    plot(t_estz, data.est_pos_z, 'Color', red)
    legend('Measured position', 'Kalman estimated position')
    xlabel('Time [s]')
    axis tight
    ylabel('Z Position [m]')
end

% ==========
% Velocities
% ==========
if figures
    figure('Name','Comparison estimated vel with num diff vel')
    hold on
    plot(t_meas, vel_FD_x, '-o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
    plot(t_est, data.est_vel_x, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
%     xlim([3.35,4.6]);
    % ylim([-0.58, -0.47]);
    % xlim(xrange_high);
    % ylim([-0.2, 0.2]);
    xlim(xrange);
    % ylim([-0.2, 0.2]);
    legend('Finite diff', 'Kalman estimation')
    xlabel('Time (s)')
    % axis tight
    ylabel('Velocity (m/s)')
end

if all_figures    
    
    % all directions
    figure('Name','Comparison estimated vel with num diff vel')

    subplot(311)
    hold on
    plot(t_meas, vel_FD_x, 'Color', blue)
    plot(t_est, data.est_vel_x, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('X Velocity [m/s]')

    subplot(312)
    hold on
    plot(t_meas, vel_FD_y, 'Color', blue)
    plot(t_est, data.est_vel_y, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('Y Velocity [m/s]')

    subplot(313)
    hold on
    plot(t_measz, vel_FD_z, 'Color', blue)
    plot(t_estz, data.est_vel_z, 'Color', red)
    legend('Finite differences', 'Kalman estimation')
    xlabel('Time [s]')
    axis tight
    ylabel('Z Velocity [m/s]')
end


% =============
% Accelerations
% =============
fs = mean(1./dt_meas);
f0 = 0.8;
fc = f0; % cutoff frequency (chosen)
fcn = fc/(fs/2); % normalized cutoff frequency (as butter() accepts)
nb = 3;
[B, A] = butter(nb,fcn);
vel_x_filt = filtfilt(B,A,vel_FD_x);
acc_FD_x = gradient(vel_x_filt)./dt_meas;
acc_x_filt = filtfilt(B,A,acc_FD_x);

if figures
        % only x
    figure('Name','Acceleration')
    hold on
    plot(t_meas, acc_FD_x, 'Color', blue)
    plot(t_meas, acc_x_filt, 'Color', red)
%     plot(t_est(1:end-1), data.est_vel_x, 'Color', red)
    legend('Finite diff','Filt')
    xlabel('Time (s)')
    axis tight
    ylabel('X Acceleration (m/s^2)')
    xlim([xrange])
end
    

if all_figures
    % only x
    figure('Name','Acceleration')
    hold on
    plot(t_meas, acc_FD_x, 'Color', blue)
    plot(t_meas, acc_x_filt, 'Color', red)
%     plot(t_est(1:end-1), data.est_vel_x, 'Color', red)
    legend('Finite diff','Filt')
    xlabel('Time (s)')
    axis tight
    ylabel('X Acceleration (m/s^2)')
end


%% Compare ZOH using only measurements VS AKF predictions
reference = interp1(t_meas,data.meas_pos_x,t_est);
ZOH = interp1(t_meas,data.meas_pos_x,t_est,'previous');
if figures
    % pos
    figure
    hold on
    plot(t_est, reference, '--o', 'MarkerSize',3.0,'Color',black,'markerfacecolor',black)
    plot(t_meas, data.meas_pos_x,'o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
    plot(t_est,ZOH,'-x','MarkerSize',7.0,'Color',yellow)
    plot(t_est, data.est_pos_x, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
    legend('Reference','Raw measurements','ZOH','AKF')
    xlim(xrange);
%     ylim([-0.723, -0.7]);
    xlabel('Time (s)')
    ylabel('Position (m)')
    yticks([1.61, 1.62, 1.63])
%     yticks([-0.72, -0.71, -0.7])  % med

end

% Calculate error of ZOH / AKF wrt reference (avg and peak)
ZOH_error = abs(ZOH - reference);
AKF_error = abs(data.est_pos_x - reference);

ZOH_error_range = ZOH_error(indexl:indexr);
AKF_error_range = AKF_error(indexl:indexr);
ZOHmean = mean(ZOH_error_range);
ZOHpeak = max(ZOH_error_range);
AKFmean = mean(AKF_error_range);
AKFpeak = max(AKF_error_range);
display(ZOHmean)
display(ZOHpeak)
display(AKFmean)
display(AKFpeak)


figure
plot(ZOH_error(indexl:indexr))
hold on
plot(AKF_error(indexl:indexr))
legend('ZOH','AKF')


%% Mooie combinatieplot
figure('Name','Comparison estimated vel with num diff vel')

subplot(3,1,[1 2])
hold on
plot(t_meas, data.meas_pos_x,'o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
plot(t_est,ZOH,'-x','MarkerSize',7.0,'Color',yellow)
plot(t_est, data.est_pos_x, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
legend('Measurements','ZOH','AKF')
xlim(xrange);
% ylim([-0.58, -0.47]);
% ylim([-0.723, -0.7]);
xlabel("Time (s)",'FontSize',11);
ylabel('Position (m)','FontSize',11);
set(gca,'FontSize',11)

subplot(3,1,3)
title({'';''})
hold on
plot(t_meas, vel_FD_x, '-o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
plot(t_est, data.est_vel_x, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
% xlim([3.35,4.6]);
% ylim([-0.58, -0.47]);
% xlim(xrange_high);
% ylim([-0.2, 0.2]);
xlim(xrange);
% ylim([-0.2, 0.2]);
legend('Finite Difference', 'AKF')
xlabel('Time (s)','FontSize',11)
% axis tight
ylabel('Velocity (m/s)','FontSize',11)
set(gca,'FontSize',11)


% z
if zfig
    reference = interp1(t_meas,data.meas_pos_x,t_est);
    ZOHz = interp1(t_meas,data.meas_pos_z,t_est,'previous');

    figure('Name','Z Comparison estimated vel with num diff vel')

    subplot(3,1,[1 2])
    hold on
    plot(t_meas, data.meas_pos_z,'o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
    plot(t_est,ZOHz,'-x','MarkerSize',7.0,'Color',yellow)
    plot(t_est, data.est_pos_z, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
    legend('Measurements','ZOH','AKF')
    xlim(xrange);
    % ylim([-0.58, -0.47]);
    % ylim([-0.723, -0.7]);
    xlabel("Time (s)",'FontSize',11);
    ylabel('Position (m)','FontSize',11);
    set(gca,'FontSize',11)

    subplot(3,1,3)
    title({'';''})
    hold on
    plot(t_meas, vel_FD_z, '-o','MarkerSize',5.0,'Color',blue,'markerfacecolor',blue)
    plot(t_est, data.est_vel_z, '-d','MarkerSize',3.0, 'Color',red,'markerfacecolor',red)
    % xlim([3.35,4.6]);
    % ylim([-0.58, -0.47]);
    % xlim(xrange_high);
    % ylim([-0.2, 0.2]);
    xlim(xrange);
    % ylim([-0.2, 0.2]);
    legend('Finite Difference', 'AKF')
    xlabel('Time (s)','FontSize',11)
    % axis tight
    ylabel('Velocity (m/s)','FontSize',11)
    set(gca,'FontSize',11)
end

