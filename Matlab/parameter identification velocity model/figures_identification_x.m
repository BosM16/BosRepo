%% IMPORTANT NOTE:
% For position plots first run Identification_x.m,
% for velocity plots run Identification_velx.m.

% Blue
% 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5
% Yellow
% 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',2.5
% Red
% 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5


%% INPUTS - OUTPUTS
figure('Name','Measurement Data')
title('Position x, input j')
subplot(311), plot(time, input, 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
ylim([-1.0 1.0])
xlabel('Time [s]')
legend('Input: j cmd [-]')
subplot(312), plot(time, output_x,'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5)
xlabel('Time [s]')
legend('Output: x position [m]')
subplot(313), plot(time, velocity_x,'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',2.5)
xlabel('Time [s]')
legend('Output: x velocity [m/s]')

%% POSITION FIT RESULT
figure('Name','Position fit result')
subplot(211)
hold on
plot(t, output_x_filt,'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5)
plot(t, x6, 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
title('Position fit: Simulation VS Filtered Measurement')
legend({'pos_{x,meas}','pos_{x,sim}'}, 'FontSize',10)

xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight
subplot(212)
plot(t,output_x_filt - x6, 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',2.5)
title('Difference between simulation and measurement')
legend({'pos_{x,meas}-pos_{x,sim}'}, 'FontSize',10)
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight

%% VELOCITY FIT RESULT
figure('Name','Velocity fit result')
subplot(211)
hold on
plot(t, velocity_x_filt,'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5)
plot(t,x4, 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
legend('v_{x,meas}', 'v_{x,sim}')
title('Velocity fit: Simulation VS Filtered Measurement')
xlabel('Time [s]')
axis tight
ylabel('Velocity [m/s]')
subplot(212)
plot(t,velocity_x_filt - x4, 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',1.5)
title('Difference between simulation and measurement')
legend('v_{x,meas}-v_{x,sim}')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
axis tight

%% VELOCITY FIT INVERSE WITH LPF

figure('Name','Difference Empirical - Continuous VS inverse filter')
subplot(2,1,1)
semilogx(f, 20*log10(abs(FRF_diff)), 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',1.5)
hold on
semilogx(f, 20*log10(abs(FRF_LPF.^(-1))), 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('magnitude  [m]')
legend('FRF_{diff}', 'LPF', 'Location','northwest')
axis tight
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(FRF_diff)), 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',1.5)
hold on
semilogx(f, 180/pi*unwrap(angle(FRF_LPF.^(-1))), 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('phase  [^\circ]')
legend('FRF_{diff}', 'LPF', 'Location','northwest')


sys_c4_eval = squeeze(freqresp(sys_c4^(-1),2*pi*f));
sys_LPF_eval = squeeze(freqresp(sys_LPF^(-1),2*pi*f));

figure('Name','Filtered vs non filtered inverse continuous time system')
subplot(2,1,1)
semilogx(f, 20*log10(abs(sys_c4_eval)), 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',1.5)
hold on
semilogx(f, 20*log10(abs(sys_LPF_eval)), 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('magnitude  [dB]')
legend('System without filtering','Filtered system', 'Location','northwest')
axis tight
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(sys_c4_eval)), 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',1.5)
hold on
semilogx(f, 180/pi*unwrap(angle(sys_LPF_eval)), 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('phase  [^\circ]')
legend('System without filtering','Filtered system', 'Location','northwest')