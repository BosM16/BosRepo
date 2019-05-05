% PID performance check
clear variables
close all
clc

run PIDdesign

% Colors
blue = [0.3010, 0.7450, 0.9330];
red = [0.6350, 0.0780, 0.1840];
yellow = [0.9290, 0.6940, 0.1250];
black = [0, 0, 0];
set(0, 'DefaultLineLineWidth', 1.5);


%% Load data
data = load("data/pid_step_input_y_10");

% preprocessing
init = 3;
timelen = length(data.meas_time);
data.meas_time = data.meas_time(init:end);
data.meas_time = data.meas_time - data.meas_time(1);
data.hover_setpoint_x = data.hover_setpoint_x(init:timelen)-data.hover_setpoint_x(init);
data.hover_setpoint_y = data.hover_setpoint_y(init:timelen)-data.hover_setpoint_y(init);
data.hover_setpoint_z = data.hover_setpoint_z(init:timelen)-data.hover_setpoint_z(init);
data.meas_pos_x = data.meas_pos_x(init:timelen)-data.meas_pos_x(init);
data.meas_pos_y = data.meas_pos_y(init:timelen)-data.meas_pos_y(init);
data.meas_pos_z = data.meas_pos_z(init:timelen)-data.meas_pos_z(init);
data.input_x = data.input_x(init:timelen);
data.input_y = data.input_y(init:timelen);
data.input_z = data.input_z(init:timelen);


% select desired time
trange = [7, data.meas_time(end)];
% trange = [2, 6];

[~,indexl] = min(abs(data.meas_time-trange(1)));
[~,indexr] = min(abs(data.meas_time-trange(2)));

%% Simulation
t = data.meas_time(1):0.02:data.meas_time(end)+0.02;
xsim = lsim(xsys_cl,data.hover_setpoint_x,t);
% figure
% plot(xsim)


%% Figure
figure('Name','PID step response')
hold on
plot(data.meas_time(indexl:indexr), data.hover_setpoint_x(indexl:indexr),'Color',blue)
plot(data.meas_time(indexl:indexr), data.meas_pos_x(indexl:indexr),'Color',red)
plot(t(indexl:indexr),xsim(indexl:indexr),'Color',yellow)
legend('Reference', 'Measured response', 'Simulated response')
xlabel('Time (s)')
ylabel('Position (m)')
axis tight







