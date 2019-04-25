% Simulation

clear variables
close all
clc

%% Identify models
run identify_params

%% make filter
Ts = 0.01;
fs = 1/Ts;
f0 = 0.8;
fc = f0; % cutoff frequency (chosen)
fcn = fc/(fs/2); % normalized cutoff frequency (as butter() accepts)
nb = 3;
[B, A] = butter(nb,fcn);

%% Simulate xmodel
t = 0:Ts:10;
% input
jmax = 1;
j_step = 0.6*jmax*(t>1);
j_ramp = (t-1)*0.5.*j_step;
j_ramp(j_ramp>jmax) = jmax;

x_step = lsim(xmodel.tf_pos,j_step,t);
x_ramp = lsim(xmodel.tf_pos,j_ramp,t);

v_step = gradient(x_step)./Ts;
v_ramp = gradient(x_ramp)./Ts;

a_step = gradient(v_step)./Ts;
a_ramp = gradient(v_ramp)./Ts;


%% Figures

% step
figure
subplot(311)
hold on
plot(t,x_step)
plot(t,j_step)
title('pos')
legend('step response','step input')

subplot(312)
hold on
plot(t,v_step)
plot(t,j_step)
title('vel')

subplot(313)
hold on
plot(t,a_step)
plot(t,j_step)
title('acc')

% ramp
figure
subplot(311)
hold on
plot(t,x_ramp)
plot(t,j_ramp)
title('pos')
legend('ramp response','ramp input')


subplot(312)
hold on
plot(t,v_ramp)
plot(t,j_ramp)
title('vel')

subplot(313)
hold on
plot(t,a_ramp)
plot(t,j_ramp)
title('acc')
