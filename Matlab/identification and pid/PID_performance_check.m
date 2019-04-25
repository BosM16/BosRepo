% PID performance check
clear variables
close all
clc

% Colors
blue = [0.3010, 0.7450, 0.9330];
red = [0.6350, 0.0780, 0.1840];
yellow = [0.9290, 0.6940, 0.1250];
black = [0, 0, 0];

%% Load data
data = load("data/pid_step_input_10");

% preprocessing
timelen = length(data.meas_time);
data.meas_time = data.meas_time(2:end);
data.meas_time = data.meas_time - data.meas_time(1);
data.hover_setpoint_x = data.hover_setpoint_x(2:timelen);
data.hover_setpoint_y = data.hover_setpoint_y(2:timelen);
data.hover_setpoint_z = data.hover_setpoint_z(2:timelen);
data.meas_pos_x = data.meas_pos_x(2:timelen);
data.meas_pos_y = data.meas_pos_y(2:timelen);
data.meas_pos_z = data.meas_pos_z(2:timelen);
data.input_x = data.input_x(2:timelen);
data.input_y = data.input_y(2:timelen);
data.input_z = data.input_z(2:timelen);


% select desired time
trange = [0, data.meas_time(end)];
% trange = [2, 6];

[~,indexl] = min(abs(data.meas_time-trange(1)));
[~,indexr] = min(abs(data.meas_time-trange(2)));


%% Figure
figure('Name','PID step response')
hold on
plot(data.meas_time, data.hover_setpoint_y)
plot(data.meas_time, data.meas_pos_y)
legend('Reference', 'Response')
xlabel('Time (s)')
ylabel('Position (m)')







