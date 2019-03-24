clear all
close all
clc
set(0, 'DefaultLineLineWidth', 1.5);

%% Load data
data = load("data/vhat_data_check");

%% Numerically differentiate position to get velocity estimation
vel_est_x = (data.pos_x(2:end) - data.pos_x(1:end - 1))*100;
vel_est_y = (data.pos_y(2:end) - data.pos_y(1:end - 1))*100;
vel_est_z = (data.pos_z(2:end) - data.pos_z(1:end - 1))*100;

data.vel_x = data.vel_x(1:end - 1);
data.vel_y = data.vel_y(1:end - 1);
data.vel_z = data.vel_z(1:end - 1);

t = 0:0.01:(length(vel_est_x) - 1)/100;

%% Plots to compare the velocity estimation of the kalman to the numericcaly differentiated velocity
figure('Name','Comparison estimated vel with num diff vel')
subplot(211)
hold on
plot(t, vel_est_x)
plot(t, data.vel_x)
legend('num differences', 'Kalman estimation')
xlabel('Time [s]')
axis tight
ylabel('X Velocity [m/s]')
hold on
plot(t, vel_est_y)
plot(t, data.vel_y)
legend('num differences', 'Kalman estimation')
xlabel('Time [s]')
axis tight
ylabel('Y Velocity [m/s]')
subplot(213)
hold on
plot(t, vel_est_z)
plot(t, data.vel_z)
legend('num differences', 'Kalman estimation')
xlabel('Time [s]')
axis tight
ylabel('Z Velocity [m/s]')