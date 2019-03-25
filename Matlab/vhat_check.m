clear all
close all
clc
set(0, 'DefaultLineLineWidth', 1.5);

%% Load data
data = load("data/vhat_data_check");
data2 = load("data/vhat_data_check2");

%% Numerically differentiate position to get velocity estimation
vel_est_x = (data.pos_x(2:end) - data.pos_x(1:end - 1))*100;
vel_est_y = (data.pos_y(2:end) - data.pos_y(1:end - 1))*100;
vel_est_z = (data2.pos_z(2:end) - data2.pos_z(1:end - 1))*100;

data.vel_x = data.vel_x(1:end - 1);
data.vel_y = data.vel_y(1:end - 1);
data2.vel_z = data2.vel_z(1:end - 1);

t = 0:0.01:(length(vel_est_x) - 1)/100;
t2 = 0:0.01:(length(vel_est_z) - 1)/100;

%% Plots to compare the velocity estimation of the kalman to the numerically differentiated velocity
figure('Name','Comparison estimated vel with num diff vel')
subplot(311)
hold on
plot(t, vel_est_x)
plot(t, data.vel_x)
legend('num differences', 'Kalman estimation')
xlabel('Time [s]')
axis tight
ylabel('X Velocity [m/s]')
subplot(312)
hold on
plot(t, vel_est_y)
plot(t, data.vel_y)
legend('num differences', 'Kalman estimation')
xlabel('Time [s]')
axis tight
ylabel('Y Velocity [m/s]')
subplot(313)
hold on
plot(t2, vel_est_z)
plot(t2, data2.vel_z)
legend('num differences', 'Kalman estimation')
xlabel('Time [s]')
axis tight
ylabel('Z Velocity [m/s]')

