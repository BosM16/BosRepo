% Asynchronous Kalman performance check.
clear variables
close all
clc
set(0, 'DefaultLineLineWidth', 1.5);

% Colors
blue = [0.3010, 0.7450, 0.9330];
red = [0.6350, 0.0780, 0.1840];
yellow = [0.9290, 0.6940, 0.1250];


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

data.est_vel_x = data.est_vel_x(1:end - 1);
data.est_vel_y = data.est_vel_y(1:end - 1);
dataz.est_vel_z = dataz.est_vel_z(1:end - 1);

t_meas = data.meas_time - data.est_time(1);
t_est  = data.est_time - data.est_time(1);

t_measz = dataz.meas_time - dataz.est_time(1);
t_estz  = dataz.est_time - dataz.est_time(1);

%% Plots to compare the velocity estimation of the kalman to the numerically differentiated velocity

% Position
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

% Velocities
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

