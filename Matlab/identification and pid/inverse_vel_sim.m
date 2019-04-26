% Simulate inverse vel model
close all
clear variables
clc

run identify_params


%% load data
data = load("data/ff_model_check2");
t = 0:0.01:(length(data.time)-1)*0.01;
jsim = lsim(xmodel.ss_vel_invLPF,data.in_x,t);
jreal = data.out_x;

figure
hold on
plot(t, jsim) 
plot(t, jreal)
xlabel('Time (s)')
ylabel('Drone input (-)')
legend()

figure 
plot(t, (jsim-jreal'))
