close all
clear variables

colors.blue   = [0.3010, 0.7450, 0.9330];
colors.red    = [0.6350, 0.0780, 0.1840];
colors.yellow = [0.9290, 0.6940, 0.1250];

%% 
s = tf('s');
sysbrol1 =  (s+1)/(1+s+s^2);
sysbrol2 = -(s-1)/(1+s+s^2);

step1 = step(sysbrol1);
step2 = step(sysbrol2);

figure
hold on
plot(step1, 'Color', colors.blue, 'LineWidth',2.5)
plot(step2, 'Color', colors.red, 'LineWidth',2.5)
set(gca,'xtick',[], 'ytick',[])
xlabel('Time')
ylabel('Amplitude')
legend('Minimum phase', 'Non-minimum phase')
