%%% Parameter estimation %%%
% zie ook 'test.m' !
clear all
close all

%% Load data from .mat-file

load angle_identification_y


%% Extract some signals from the data

set(0, 'DefaultLineLineWidth', 1);


% Cutoff useful data
% first_index = find(input~=0, 1);
input = input(1:end-10)';
output_x = output_x(1:end-10)';
output_y = output_y(1:end-10)';
output_z = output_z(1:end-10)';

dt = 0.02;
time = (0:dt:(length(input)-1)*dt)';

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

subplot(325), plot(time, output_z), title('Position z'), xlabel('time [s]'), ylabel('position [m]')
subplot(326), plot(time, velocity_z), title('Velocity z'), xlabel('time [s]'), ylabel('speed [m/s]')

% Input signal
figure('Name', 'Input signal')
plot(time, input)
xlabel('time [s]')
ylabel('Amplitude [-]')
axis([time(1)-1 time(end)+1 -1 1])


%% Frequency domain
Ts = dt;
fs = 1/dt;
N = length(velocity_y);
t = [0:N-1]'*Ts;
N = numel(input);
f = [0:N-1]'*(fs/N);

input_f = fft(input);
output_x_f = fft(output_x);
output_y_f = fft(output_y);
velocity_x_f = fft(velocity_x);
velocity_y_f = fft(velocity_y);

%indices = (nop+1):nop:(numel(f)/2); % alle 0 waarden er uit halen zodat niet delen door 0?
%f = f(indices);
%input_f = input_f(indices);
%velocity_x_f = velocity_x_f(indices);
%velocity_y_f = velocity_y_f(indices);

FRF = velocity_y_f./input_f;

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
f0 = 0.5; % crossover frequency
fc = 5*f0; % cutoff frequency
fcn = fc/(fs/2); % normalized cutoff frequency


%% Filtering of the in- and output data using Butterworth filter
[B, A] = butter(3,fcn); % order must be higher than order of system, 
                             % adjust cut-off frquency to be higher than highest eigenfrequency of the system
                             % data sampling at 100Hz

% input filtering                           
input_filt = filter(B,A,input);
% output filtering
velocity_x_filt = filter(B,A,velocity_x);
velocity_y_filt = filter(B,A,velocity_y);

figure('Name','filtered input input')
plot(t,input,t,input_filt),title('Input input filtered')

figure('Name','filtered output measurement')
subplot(211), plot(time, velocity_x, time, velocity_x_filt),title('v_{x,filt}')
subplot(212), plot(time, velocity_y, time, velocity_y_filt),title('v_{y,filt}')

%% Least squares solution for approximation of the parameters in the system
% Without filtering
% 2nd order proper
% y[k] = -a1*y[k-1]-a0*y[k-2]+b2*u[k]+b1*u[k-1]+b0*u[k-2]
y = velocity_y(3:end);
Phi = [-velocity_y(2:end-1), -velocity_y(1:end-2), input(3:end), input(2:end-1), input(1:end-2)];
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
plot(t, velocity_y,'g')
plot(t, x1)
title('2nd - no filter - proper: Simulation vs Measurement')
legend('v_{y,meas}','v_{y,sim}')
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight
subplot(212)
plot(t,velocity_y - x1)
title('Difference between simulation and measurement')
legend('v_{y,meas}-v_{y,sim}')
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight

figure('Name','2nd - no filter - proper: Pole Zero Map'),pzmap(sys_d1)


%% With Butterworth filtering of in and output
% 2nd order proper
y2 = velocity_y_filt(3:end);
Phi2 = [-velocity_y_filt(2:end-1), -velocity_y_filt(1:end-2), input_filt(3:end), input_filt(2:end-1), input_filt(1:end-2)];
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
plot(t, velocity_y,'g')
plot(t, velocity_y_filt)
plot(t,x2)
legend('v_{y,meas}', 'v_{y,filt}', 'v_{y,sim}')
title('2nd - filtered - proper: Simulation vs Measurement')
xlabel('Time [s]')
axis tight
ylabel('Velocity [m/s]')
subplot(212)
plot(t,velocity_y - x2)
title('Difference between simulation and measurement')
legend('v_{y,meas}-v_{y,sim}')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
axis tight

figure('Name','2nd - filtered - proper: Pole Zero Map'),pzmap(sys_d2)

%% With Butterworth filtering of in and output
% 2nd order strictly proper

y3 = velocity_y_filt(3:end);
Phi3 = [-velocity_y_filt(2:end-1), -velocity_y_filt(1:end-2), input_filt(2:end-1), input_filt(1:end-2)];
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
plot(t, velocity_y,'g')
plot(t, velocity_y_filt)
plot(t,x3)
legend('v_{y,meas}', 'v_{y,filt}', 'v_{y,sim}')
title('2nd - filtered - strictly proper: Simulation vs Measurement')
xlabel('Time [s]')
axis tight
ylabel('Velocity [m/s]')
subplot(212)
plot(t,velocity_y - x3)
title('Difference between simulation and measurement')
legend('v_{y,meas}-v_{y,sim}')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
axis tight

figure('Name','2nd - filtered - strictly proper: Pole Zero Map')
pzmap(sys_d3)



%% ========================================================================
%                               MINIMUM PHASE
%  ========================================================================
%% With Butterworth filtering of in and output
% 2nd order strictly proper minimum phase

%                       b1
%      HVxJ(z) = -----------------
%                 z^2 + a1*z + a0

y4 = velocity_y_filt(3:end);
Phi4 = [-velocity_y_filt(2:end-1), -velocity_y_filt(1:end-2), input_filt(1:end-2)];
theta_filt4 = Phi4\y4;

B4 = [theta_filt4(3)];
A4 = [1, theta_filt4(1) theta_filt4(2)];

sys_d4 = tf(B4, A4, Ts);

FRF4 = squeeze(freqresp(sys_d4,2*pi*f));

figure('Name','2nd - filtered - strictly proper - Minimum Phase: Freq Response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF4)))
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

figure('Name','2nd - filtered - strictly proper - Minimum Phase: Simulation')
subplot(211)
hold on
plot(t, velocity_y,'g')
plot(t, velocity_y_filt)
plot(t,x4)
legend('v_{y,meas}', 'v_{y,filt}', 'v_{y,sim}')
title('2nd - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
xlabel('Time [s]')
axis tight
ylabel('Velocity [m/s]')
subplot(212)
plot(t,velocity_y - x4)
title('Difference between simulation and measurement')
legend('v_{y,meas}-v_{y,sim}')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
axis tight

figure('Name','2nd - filtered - strictly proper - Minimum Phase: Pole Zero Map')
pzmap(sys_d4)



%% Comparison
figure('Name','Comparison frequency responses')
hold on
bode(sys_d1), grid
bode(sys_d2)
bode(sys_d3)
bode(sys_d4)
legend('2nd - not filtered - proper', ...
       '2nd - filtered - proper', ...
       '2nd - filtered - strictly proper', ...
       '2nd - filtered - proper - minimum phase')

figure('Name', 'Comparison simulation results')
plot(t,[velocity_y velocity_y_filt x1 x2 x3 x4])
legend('measured', ...
       'filtered measurement', ...
       '2nd - no filter - proper', ...
       '2nd - filter - proper', ...
       '2nd - filter - strictly proper', ...
       '2nd - filter - strictly proper - minimum phase')

%% Find crossover frequency of best fit
[error, index] = min(abs(20*log10(abs(FRF3(1:end-100)))));
f0 = f(index);

fprintf('f0x: %d \n', f0)


%% continuous time transfer function  + 50Hz --> 100Hz

% 2nd filtered strictly proper
sys_c3 = d2c(sys_d3);
sys_3 = c2d(sys_c3, 0.01);

% 2nd filtered strictly proper MINIMUM PHASE
% NOTE: we use 'matched' in order to preserve minimum phase in the
% conversion between discrete and continuous.
sys_c4 = d2c(sys_d4,'matched')
sys_4 = c2d(sys_c4, 0.01,'matched')

figure('Name','PZmaps for minimum-phase discrete & continuous')
hold on
pzmap(sys_d4)
pzmap(sys_4)
pzmap(sys_c4)

legend('50Hz','100Hz','continuous')

%% Save result (transfer function)
% sys_2nd = sys_dpk;
% sys_2nd_f = sys_d2;
% sys_c2nd = sys_cpk;
% save('HVJ_y','sys_2nd')
% save('HVJ_y_filtered', 'sys_2nd_f')
% save('HVJ_y_cont','sys_c2nd')

%% Plot 100Hz fit
%  & Integreer deze en kijk of fit op positie goed is.
dt100Hz = .01;
t100Hz = (0:dt100Hz:(length(input)-1)*dt)';

% Differentiator
z = tf('z', dt100Hz);
int_d = z*dt100Hz/(z-1);
sys_x = int_d * sys_4;

% Interpolate input
input100Hz = interp1(t,input,t100Hz);

v_sim = lsim(sys_4,input100Hz,t100Hz);
x_sim = lsim(sys_x,input100Hz,t100Hz);

figure('Name','Velocity 100Hz model simulation')
plot(t,velocity_y_filt, t100Hz, v_sim)
legend('filtered velocity','simulation')

figure('Name', 'Integrated velocity simulation')
plot(t, output_y, t100Hz, x_sim);
% Conclusie: komt heel goed overeen met fit 3e orde op positie!

