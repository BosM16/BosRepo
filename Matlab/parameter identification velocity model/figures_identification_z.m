%% IMPORTANT NOTE:
% For position plots first run Identification_z.m,
% for velocity plots run Identification_velz.m.

% Blue
% 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5
% Yellow
% 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',2.5;
% Red
% 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5


%% INPUTS - OUTPUTS
figure('Name','Measurement Data')
title('Position z, input j')
subplot(311), plot(time, input, 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
ylim([-1.0 1.0])
xlabel('Time [s]')
legend('Input: j cmd [-]')
subplot(312), plot(time, output_z,'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5)
xlabel('Time [s]')
legend('Output: z position [m]')
subplot(313), plot(time, velocity_z,'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',2.5)
xlabel('Time [s]')
legend('Output: z velocity [m/s]')

%% POSITION FIT RESULT
figure('Name','Position fit')
subplot(211)
hold on
plot(t, output_z_filt,'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5)
plot(t, x0, 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
title('Position fit: Simulation VS Filtered Measurement')
legend({'pos_{z,meas}','pos_{z,sim}'}, 'FontSize',10)
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight
subplot(212)
plot(t,output_z_filt - x0, 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',2.5)
title('Difference between simulation and measurement')
legend({'pos_{z,meas}-pos_{z,sim}'}, 'FontSize',10)
xlabel('Time [s]')
ylabel('Displacement [m]')
axis tight


%% VELOCITY FIT RESULT
figure('Name','Velocity fit result')
subplot(211)
hold on
plot(t, velocity_z_filt,'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5)
plot(t,x0, 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
legend('v_{z,meas}', 'v_{z,sim}')
title('Velocity fit: Simulation VS Filtered Measurement')
xlabel('Time [s]')
axis tight
ylabel('Velocity [m/s]')
subplot(212)
plot(t,velocity_z_filt - x0, 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',1.5)
title('Difference between simulation and measurement')
legend('v_{z,meas}-v_{z,sim}')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
axis tight
