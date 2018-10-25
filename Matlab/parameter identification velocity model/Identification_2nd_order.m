%%% Parameter estimation %%%
% zie ook 'test.m' !
clear all
close all

%% Load data from .mat-file
load experiment_blok_lang_5000
load experiment_grond_blok_lang_5000

%% What is sent by QRC on which channel:
%rec_bl_5000_air.labels

%% Extract some signals from the data
% zorg dat deze namen overeenkomen met de channels in QRobotics!
% of pas de labels aan in 'rec'
set(0, 'DefaultLineLineWidth', 1);
encoder1 = rec_bl_5000_air.getData('enc1_value');
encoder2 = rec_bl_5000_air.getData('enc2_value');
pulse    = rec_bl_5000_air.getData('Pulse');
% encoder1 = rec_bl_5000_ground.getData('enc1_value');
% encoder2 = rec_bl_5000_ground.getData('enc2_value');
% pulse    = rec_bl_5000_ground.getData('Pulse');
%pulse is niet het echte inputsignaal in volt --> herwerkt naar echte input
for i=1:numel(pulse)
    if pulse(i) == 1
       pulse(i) = -5; 
    
    elseif pulse(i) == 2
       pulse(i) = 5;
    end
    
end 
first_index = find(pulse~=0, 1);
pulse = pulse(first_index:2000);
encoder1 = encoder1(first_index:2000);
encoder2 = encoder2(first_index:2000);

time     = rec_bl_5000_air.getData('time')*10^-3;
%time     = rec_bl_5000_ground.getData('time')*10^-3;
time = time(first_index:2000) - time(first_index);

% Do some plotting 

figure('Name','Encoder position')
subplot(211), plot(time, encoder1), title('Encoder position 1'), xlabel('Time [s]'), ylabel('Encoder position [rad]'), xlim([0,time(end)])
subplot(212), plot(time, encoder2), title('Encoder position 2'), xlabel('Time [s]'), ylabel('Encoder position [rad]'), xlim([0,time(end)])
% differentiation of encoder position
% timestep (gradient assumes timestep 1)
dt = 0.01; % vervang dit door t(2)-t(1), manual because of bad time vector in Qrobotics
enc1_speed = gradient(encoder1)/dt;
enc2_speed = gradient(encoder2)/dt;

figure('Name','Angular velocity')
subplot(211), plot(time, enc1_speed), title('Angular velocity 1'), xlabel('Time [s]'), ylabel('Angular velocity [rad/s]'), xlim([0,time(end)])
subplot(212), plot(time, enc2_speed), title('Angular velocity 2'), xlabel('Time [s]'), ylabel('Angular velocity [rad/s]'), xlim([0,time(end)])

% input
figure('Name', 'Input signal block pulse')
plot(time, pulse)
title('Input signal block pulse')
xlabel('Time [s]')
ylabel('Voltage [V]')
xlim([0,time(end)])
ylim([-6,6])

%% Frequency domain
fs = 100;
Ts = 1/fs;
N = length(enc1_speed);
t = [0:N-1]'*Ts;
N = numel(pulse);
f = [0:N-1]'*(fs/N);

pulse_f = fft(pulse);
encoder1_f = fft(encoder1);
encoder2_f = fft(encoder2);
enc1_speed_f = fft(enc1_speed);
enc2_speed_f = fft(enc2_speed);

%indices = (nop+1):nop:(numel(f)/2); % alle 0 waarden er uit halen zodat niet delen door 0?
%f = f(indices);
%pulse_f = pulse_f(indices);
%enc1_speed_f = enc1_speed_f(indices);
%enc2_speed_f = enc2_speed_f(indices);

FRF1 = enc1_speed_f./pulse_f;
FRF2 = enc2_speed_f./pulse_f;

axis tight
figure('Name', 'empirical transfer function freq response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF1)), 'LineWidth', 1)
grid on
xlabel('f [Hz]')
xlim([f(1) f(end)])
ylabel('|FRF| [m]')
subplot(2,1,2),semilogx(f, 180/pi*unwrap(angle(FRF1)), 'LineWidth', 1)
grid on
axis tight
xlabel('f  [Hz]')
ylabel('\phi(FRF) [^\circ]')
xlim([f(1) f(end)])

%% Filtering of the in- and output data using Butterworth filter
[B, A] = butter(3,0.1*5); % order must be higher than order of system, 
                             % adjust cut-off frquency to be higher than highest eigenfrequency of the system
                             % data sampling at 100Hz

% input filtering                           
pulse_filt = filter(B,A,pulse);
% output filtering
enc1_speed_filt = filter(B,A,enc1_speed);
enc2_speed_filt = filter(B,A,enc2_speed);

figure('Name','filtered input pulse')
plot(t,pulse,t,pulse_filt),title('Input pulse filtered')

figure('Name','filtered output measurement')
subplot(211), plot(time, enc1_speed, time, enc1_speed_filt),title('enc1\_speed_{filt}')
subplot(212), plot(time, enc2_speed, time, enc2_speed_filt),title('enc2\_speed_{filt}')

%% Least squares solution for approximation of the parameters in the system
% Without filtering
% y[k] = -a1*y[k-1]-a0*y[k-2]+b2*u[k]+b1*u[k-1]+b0*u[k-2]
y = enc1_speed(3:end);
Phi = [-enc1_speed(2:end-1), -enc1_speed(1:end-2), pulse(3:end), pulse(2:end-1), pulse(1:end-2)];
theta = Phi\y;

B1 = [theta(3), theta(4), theta(5)];
A1 = [1, theta(1) theta(2)];

sys_d1 = tf(B1, A1, Ts);

FRF1 = squeeze(freqresp(sys_d1,2*pi*f));

figure('Name','NOT filtered Identified transfer funtion freq response'), subplot(211)
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

x1 = lsim(sys_d1,pulse,t);

figure('Name','lsim NOT filtered Identified transfer function')
subplot(211)
hold on
plot(t, enc1_speed,'g')
plot(t, x1)
title('Non filtered identified transfer function vs measurement')
legend('enc1\_speed_{meas}','enc1\_speed_{sim}')
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight
subplot(212)
plot(t,enc1_speed - x1)
title('Difference between simulation and measurement')
legend('enc1\_speed_{meas}-enc1\_speed_{sim}')
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight

figure('Name','NOT  filtered pole zero map'),pzmap(sys_d1)
%% With Butterworth filtering of in and output

y2 = enc1_speed_filt(3:end);
Phi2 = [-enc1_speed_filt(2:end-1), -enc1_speed_filt(1:end-2), pulse_filt(3:end), pulse_filt(2:end-1), pulse_filt(1:end-2)];
theta_filt = Phi2\y2;

B2 = [theta_filt(3),theta_filt(4),theta_filt(5)];
A2 = [1, theta_filt(1) theta_filt(2)];

sys_d2 = tf(B2, A2, Ts);

FRF2 = squeeze(freqresp(sys_d2,2*pi*f));

figure('Name','Butterworth filtered Identified tf freq response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF2)))
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

x2 = lsim(sys_d2,pulse,t);

figure('Name','Butterworth filtered lsim time response')
subplot(211)
hold on
plot(t,enc1_speed,'g')
plot(t,x2)
legend('enc1\_speed_{meas}','enc1\_speed_{sim}')
title('Butterworth filtered identified transfer function vs measurement')
xlabel('Time [s]')
axis tight
ylabel('Displacement [m]')
subplot(212)
plot(t,enc1_speed - x2)
title('Difference between simulation and measurement')
legend('enc1\_speed_{meas}-enc1\_speed_{sim}')
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight

figure('Name','Butterworth filtered Identified tf pole-zero map'),pzmap(sys_d2)

%% Taking into account KNOWN PART of the system! (b1 & b0 = 0)
% theta(4) & theta(5) (= b1 & b0) turn out to be very small. But we know
% (from physical insight) that they are EXACTLY 0!
% pk ~ partially known
% y[k] = -a1*y[k-1] - a0*y[k-2] + b2*u[k]
y_pk = enc1_speed_filt(3:end);

Phi_pk = [-enc1_speed_filt(2:end-1), -enc1_speed_filt(1:end-2), pulse_filt(3:end)];
theta_pk = Phi_pk\y_pk;

% known part = z^2/1
B_k = [1, 0, 0];
A_k = [0, 0, 1];
sys_k = tf(B_k,A_k,Ts);

B_u = theta_pk(3);
A_u = [1, theta_pk(1) theta_pk(2)];
sys_u = tf(B_u,A_u, Ts);

sys_dpk = sys_k*sys_u;

FRF_pk = squeeze(freqresp(sys_dpk,2*pi*f));

figure('Name','Butter. filtered + partially known freq response'),subplot(211),semilogx(f, 20*log10(abs(FRF_pk)))
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('|FRF_{pk}|  [m]')
axis tight
subplot(212)
semilogx(f, 180/pi*unwrap(angle(FRF_pk)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('\phi(FRF_{pk})  [^\circ]')

x_pk = lsim(sys_dpk,pulse,t);

figure('Name','Butter. filtered + partially known lsim time response')
subplot(211)
hold on
plot(t,enc1_speed, 'g')
plot(t,x_pk)
legend('enc1\_speed_{meas}','enc1\_speed_{sim}')
xlabel('Time [s]')
title('Butterworth filtered + partially known identified transfer function vs measurement')
axis tight
ylabel('Displacement [m]')
subplot(212)
plot(t,enc1_speed - x_pk)
title('Difference between simulation and measurement')
legend('enc1\_speed_{meas}-enc1\_speed_{sim}')
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight

figure('Name','Partially known syst: pole zero map'),pzmap(sys_dpk)

%% Comparison
figure

bode(sys_d1), grid
hold on
bode(sys_d2)
hold on
bode(sys_dpk)
legend('not filtered', 'filtered', 'partially known')

figure
plot(t,[enc1_speed x1 x2 x_pk])
%% continuous equivalent of tf (pk)
a1 = theta_pk(1);
a0 = theta_pk(2);
b2 = theta_pk(3);

s = tf('s');
sys_cpk = (b2/(a0*Ts^2))/(s^2+(-a0*2*Ts-Ts*a1)/(a0*Ts^2)*s+(1+a1+a0)/(a0*Ts^2));
figure('Name','Continuous tf 2nd order')
bode(sys_cpk)
hold on
bode(sys_dpk)
legend('cont','discr')
%% Save result (transfer function)
sys_2nd = sys_dpk;
sys_2nd_f = sys_d2;
sys_c2nd = sys_cpk;
save('Hv_speed','sys_2nd')
save('Hv_speed_filtered', 'sys_2nd_f')
save('Hv_speed_c','sys_c2nd')

