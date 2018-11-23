%%% Parameter estimation %%%
% zie ook 'test.m' !
clear all
close all

%% Load data from .mat-file

load vel_identification_z_short


%% Extract some signals from the data

set(0, 'DefaultLineLineWidth', 1);


% Cutoff useful data
% first_index = find(input~=0, 1);
% input = input(1:end-10)';
% output_x = output_x(1:end-10)';
% output_y = output_y(1:end-10)';
% output_z = output_z(1:end-10)';
input = input(1:end-100)';
output_x = output_x(1:end-100)';
output_y = output_y(1:end-100)';
output_z = output_z(1:end-100)';
output_z = output_z - output_z(1);

dt = 0.02;
time = 0:dt:(length(input)-1)*dt;

% Differentiation of x-, y-, z-position
% dt = sample time (gradient assumes timestep 1)
velocity_x = gradient(output_x)/dt;
velocity_y = gradient(output_y)/dt;
velocity_z = gradient(output_z)/dt;

% Do some plotting of the measurement data 
figure('Name','Output Measurements')
subplot(321), plot(time, output_x), title('Position x'), xlabel('time [s]'), ylabel('position [m]')
subplot(322), plot(time, velocity_x), title('Velocity x'), xlabel('time [s]'), ylabel('speed [m/s]')

subplot(323), plot(time, output_y), title('Position y'), xlabel('time [s]'), ylabel('position [m]')
subplot(324), plot(time, velocity_y), title('Velocity y'), xlabel('time [s]'), ylabel('speed [m/s]')

subplot(325), plot(time, output_z, time, input), title('Position z'), xlabel('time [s]'), ylabel('position [m]')
subplot(326), plot(time, velocity_z, time, input), title('Velocity z'), xlabel('time [s]'), ylabel('speed [m/s]')

% Input signal
figure('Name', 'Input signal')
plot(time, input)
xlabel('time [s]')
ylabel('Amplitude [-]')
axis([time(1)-1 time(end)+1 -1 1])


%% Frequency domain
Ts = dt;
fs = 1/dt;
N = length(velocity_x);
t = [0:N-1]'*Ts;
N = numel(input);
f = [0:N-1]'*(fs/N);

input_f = fft(input);
output_z_f = fft(output_z);
velocity_z_f = fft(velocity_z);

FRF = output_z_f./input_f;

figure('Name', 'Empirical transfer function freq response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF)), 'LineWidth', 1)
axis tight
grid on
xlabel('f [Hz]')
xlim([f(1) f(end)])
ylabel('|FRF| [m]')
subplot(2,1,2),semilogx(f, 180/pi*unwrap(angle(FRF)), 'LineWidth', 1)
grid on
axis tight
xlabel('f  [Hz]')
ylabel('\phi(FRF) [^\circ]')
xlim([f(1) f(end)])


%% Choose a cutoff frequency for Butterworth filtering
f0 = 1.5; % crossover frequency
fc = 5*f0; % cutoff frequency
fcn = fc/(fs/2); % normalized cutoff frequency


%% Filtering of the in- and output data using Butterworth filter
[B, A] = butter(2,fcn); % order must be higher than order of system, 
                             % adjust cut-off frquency to be higher than highest eigenfrequency of the system

% input filtering                           
input_filt = filter(B,A,input);
% output filtering
output_z_filt = filter(B,A,output_z);
velocity_z_filt = filter(B,A,velocity_z);

figure('Name','filtered input')
plot(t,input,t,input_filt),title('Input filtered')

figure('Name','filtered output measurement')
plot(time, output_z, time, output_z_filt),title('pos_{z,filt}')
legend('output','filtered output')


%% Least squares solution for approximation of the parameters in the system
% With Butterworth filtering of in and output
% 1st order strictly proper

y0 = velocity_z_filt(2:end);
Phi0 = [-velocity_z_filt(1:end-1), input_filt(1:end-1)];
theta_filt0 = Phi0\y0;

B0 = [theta_filt0(2)];
A0 = [1, theta_filt0(1)];

sys_d0 = tf(B0, A0, Ts);

FRF0 = squeeze(freqresp(sys_d0,2*pi*f));

figure('Name','1st - filtered - strictly proper: Freq Response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF0)))
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('|FRF0|  [m]')
axis tight
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(FRF0)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('\phi(FRF0)  [^\circ]')

x0 = lsim(sys_d0,input,t);

figure('Name','1st - filtered - strictly proper: Simulation')
subplot(211)
hold on
plot(t, velocity_z,'g')
plot(t, velocity_z_filt)
plot(t,x0)
legend('pos_{z,meas}', 'pos_{z,filt}', 'pos_{z,sim}')
title('1st - filtered - strictly proper: Simulation vs Measurement')
xlabel('Time [s]')
axis tight
ylabel('velocity [m/s]')
subplot(212)
plot(t,velocity_z - x0)
title('Difference between simulation and measurement')
legend('pos_{z,meas}-pos_{z,sim}')
xlabel('Time [s]')
ylabel('velocity [m/s]')
axis tight

figure('Name','1st - filtered - strictly proper: Pole Zero Map'),pzmap(sys_d0)


%% ----------------------
%   Second order fitting
%  ----------------------
%% Filtering of the in- and output data using Butterworth filter
[B, A] = butter(3,fcn); % order must be higher than order of system, 
                             % adjust cut-off frquency to be higher than highest eigenfrequency of the system

% input filtering                           
input_filt = filter(B,A,input);
% output filtering
output_z_filt = filter(B,A,output_z);
velocity_z_filt = filter(B,A,velocity_z);

figure('Name','filtered input')
plot(t,input,t,input_filt),title('Input filtered')

figure('Name','filtered output measurement')
plot(time, output_z, time, output_z_filt),title('pos_{z,filt}')
legend('output','filtered output')


%% 2nd order proper
% Without filtering
% y[k] = -a1*y[k-1]-a0*y[k-2]+b2*u[k]+b1*u[k-1]+b0*u[k-2]
y = velocity_z(3:end);
Phi = [-velocity_z(2:end-1), -velocity_z(1:end-2), input(3:end), input(2:end-1), input(1:end-2)];
theta = Phi\y;

B1 = [theta(3), theta(4), theta(5)];
A1 = [1, theta(1) theta(2)];

sys_d1 = tf(B1, A1, Ts);

FRF1 = squeeze(freqresp(sys_d1,2*pi*f));

figure('Name','2nd - no filter - proper: Freq Response'), subplot(211)
semilogx(f, 20*log10(abs(FRF1)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('|FRF1|  [m]')
subplot(212),semilogx(f, 180/pi*unwrap(angle(FRF1)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('\phi(FRF1)  [^\circ]')

x1 = lsim(sys_d1,input,t);

figure('Name','2nd - no filter - proper: Simulation')
subplot(211)
hold on
plot(t, velocity_z,'g')
plot(t, x1)
title('2nd - no filter - proper: Simulation vs Measurement')
legend('pos_{z,meas}','pos_{z,sim}')
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight
subplot(212)
plot(t,velocity_z - x1)
title('Difference between simulation and measurement')
legend('pos_{z,meas}-pos_{z,sim}')
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight

figure('Name','2nd - no filter - proper: Pole Zero Map'),pzmap(sys_d1)



%% With Butterworth filtering of in and output
% Second order proper

y2 = velocity_z_filt(3:end);
Phi2 = [-velocity_z_filt(2:end-1), -velocity_z_filt(1:end-2), input_filt(3:end), input_filt(2:end-1), input_filt(1:end-2)];
theta_filt = Phi2\y2;

B2 = [theta_filt(3),theta_filt(4),theta_filt(5)];
A2 = [1, theta_filt(1) theta_filt(2)];

sys_d2 = tf(B2, A2, Ts);

FRF2 = squeeze(freqresp(sys_d2,2*pi*f));

figure('Name','2nd - filtered - proper: Freq Response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF2)))
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('|FRF2|  [m]')
axis tight
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(FRF2)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('\phi(FRF2)  [^\circ]')

x2 = lsim(sys_d2,input,t);

figure('Name','2nd - filtered - proper: Simulation')
subplot(211)
hold on
plot(t, velocity_z,'g')
plot(t, velocity_z_filt)
plot(t,x2)
legend('pos_{z,meas}', 'pos_{z,filt}', 'pos_{z,sim}')
title('2nd - filtered - proper: Simulation vs Measurement')
xlabel('Time [s]')
axis tight
ylabel('velocity [m/s]')
subplot(212)
plot(t,velocity_z - x2)
title('Difference between simulation and measurement')
legend('pos_{z,meas}-pos_{z,sim}')
xlabel('Time [s]')
ylabel('velocity [m/s]')
axis tight

figure('Name','2nd - filtered - proper: Pole Zero Map'),pzmap(sys_d2)

%% With Butterworth filtering of in and output
% 2nd order strictly proper

y3 = velocity_z_filt(3:end);
Phi3 = [-velocity_z_filt(2:end-1), -velocity_z_filt(1:end-2), input_filt(2:end-1), input_filt(1:end-2)];
theta_filt3 = Phi3\y3;

B3 = [theta_filt3(3),theta_filt3(4)];
A3 = [1, theta_filt3(1) theta_filt3(2)];

sys_d3 = tf(B3, A3, Ts);

FRF3 = squeeze(freqresp(sys_d3,2*pi*f));

figure('Name','2nd - filtered - strictly proper: Freq Response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF3)))
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('|FRF3|  [m]')
axis tight
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(FRF3)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('\phi(FRF3)  [^\circ]')

x3 = lsim(sys_d3,input,t);

figure('Name','2nd - filtered - strictly proper: Simulation')
subplot(211)
hold on
plot(t, velocity_z,'g')
plot(t, velocity_z_filt)
plot(t,x3)
legend('pos_{z,meas}', 'pos_{z,filt}', 'pos_{z,sim}')
title('2nd - filtered - strictly proper: Simulation vs Measurement')
xlabel('Time [s]')
axis tight
ylabel('velocity [m/s]')
subplot(212)
plot(t,velocity_z - x3)
title('Difference between simulation and measurement')
legend('pos_{z,meas}-pos_{z,sim}')
xlabel('Time [s]')
ylabel('velocity [m/s]')
axis tight

figure('Name','2nd - filtered - strictly proper: Pole Zero Map'),pzmap(sys_d3)


%% ----------------------
%   Third order fitting
%  ----------------------

%% Filtering of the in- and output data using Butterworth filter
[B, A] = butter(4,fcn); % order must be higher than order of system, 
                             % adjust cut-off frquency to be higher than highest eigenfrequency of the system

% input filtering                           
input_filt = filter(B,A,input);
% output filtering
output_z_filt = filter(B,A,output_z);
velocity_z_filt = filter(B,A,velocity_z);

figure('Name','filtered input')
plot(t,input,t,input_filt),title('Input filtered')

figure('Name','filtered output measurement')
plot(time, output_z, time, output_z_filt),title('pos_{z,filt}')
legend('output','filtered output')


%% With Butterworth filtering of in and output
% 3d order proper

y4 = velocity_z_filt(4:end);
Phi4 = [-velocity_z_filt(3:end-1), -velocity_z_filt(2:end-2), -velocity_z_filt(1:end-3), input_filt(4:end), input_filt(3:end-1), input_filt(2:end-2), input_filt(1:end-3)];
theta_filt4 = Phi4\y4;

B4 = [theta_filt4(4),theta_filt4(5),theta_filt4(6), theta_filt4(7)];
A4 = [1, theta_filt4(1) theta_filt4(2), theta_filt4(3)];

sys_d4 = tf(B4, A4, Ts);

FRF4 = squeeze(freqresp(sys_d4,2*pi*f));

figure('Name','3d - filtered - proper: Freq response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF4)))
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('|FRF4|  [m]')
axis tight
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(FRF4)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('\phi(FRF4)  [^\circ]')

x4 = lsim(sys_d4,input,t);

figure('Name','3d - filtered - proper: Simulation')
subplot(211)
hold on
plot(t, velocity_z,'g')
plot(t, velocity_z_filt)
plot(t,x4)
legend('pos_{z,meas}', 'pos_{z,filt}', 'pos_{z,sim}')
title('3d - filtered - proper: Simulation vs Measurement')
xlabel('Time [s]')
axis tight
ylabel('velocity [m/s]')
subplot(212)
plot(t,velocity_z - x4)
title('Difference between simulation and measurement')
legend('pos_{z,meas}-pos_{z,sim}')
xlabel('Time [s]')
ylabel('velocity [m/s]')
axis tight

figure('Name','3d - filtered - proper: Pole Zero Map'),pzmap(sys_d4)


%% With Butterworth filtering of in and output
% 3d order strictly proper

y5 = velocity_z_filt(4:end);
Phi5 = [-velocity_z_filt(3:end-1), -velocity_z_filt(2:end-2), -velocity_z_filt(1:end-3), input_filt(3:end-1), input_filt(2:end-2), input_filt(1:end-3)];
theta_filt5 = Phi5\y5;

B5 = [theta_filt5(4),theta_filt5(5),theta_filt5(6)];
A5 = [1, theta_filt5(1) theta_filt5(2), theta_filt5(3)];

sys_d5 = tf(B5, A5, Ts);

FRF5 = squeeze(freqresp(sys_d5,2*pi*f));

figure('Name','3d - filtered - strictly proper: Freq Response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF5)))
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('|FRF5|  [m]')
axis tight
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(FRF5)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('\phi(FRF5)  [^\circ]')

x5= lsim(sys_d5,input,t);

figure('Name','3d - filtered - strictly proper: Simulation')
subplot(211)
hold on
plot(t, velocity_z,'g')
plot(t, velocity_z_filt)
plot(t,x5)
legend('pos_{z,meas}', 'pos_{z,filt}', 'pos_{z,sim}')
title('3d - filtered - strictly proper: Simulation vs Measurement')
xlabel('Time [s]')
axis tight
ylabel('velocity [m/s]')
subplot(212)
plot(t,velocity_z - x5)
title('Difference between simulation and measurement')
legend('pos_{z,meas}-pos_{z,sim}')
xlabel('Time [s]')
ylabel('velocity [m/s]')
axis tight

figure('Name','3d - filtered - strictly proper: Pole Zero Map'),pzmap(sys_d5)


%% Find crossover frequency of best fit
[error, index] = min(abs(20*log10(abs(FRF3(1:end-100)))));
f0 = f(index);

fprintf('f0x: %d \n', f0)


%% continuous
sys_c0 = d2c(sys_d0)
sys_0 = c2d(sys_c0, 0.01)
