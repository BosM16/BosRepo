close all

% Important note: first run file "Identification_velx.m"
%% 
s = tf('s');
sysbrol1 =  (s+1)/(1+s+s^2);
sysbrol2 = -(s-1)/(1+s+s^2);

step1 = step(sysbrol1);
step2 = step(sysbrol2);

figure
hold on
plot(step1, 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5)
plot(step2, 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
xlabel('Time')
ylabel('Step response')
legend('Minimum phase', 'Non-minimum phase')
