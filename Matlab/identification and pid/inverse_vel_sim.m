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


%% manual simulation
% A = xmodel.ss_vel_invLPF.A;
% B = xmodel.ss_vel_invLPF.B;
% C = xmodel.ss_vel_invLPF.C;
% D = xmodel.ss_vel_invLPF.D;

A = [   2.924615161772681  -1.426022503893993   0.927378249329201;
   2.000000000000000                   0                   0;
                   0   0.500000000000000                   0;];

B = [0.25000000000000000;
     0.0;
     0.0];
C = [  0.093794142767462  -0.091022743092107   0.088262872564127];
D = 0.011815313012427;

% [A B C D] = ssdata(xmodel.ss_vel_invLPF);
% fprintf('%d',A(1,1))
% fprintf('%d',A(1,2))
% fprintf('%d',A(1,2))


u = data.in_x;
x = zeros(3, length(t));
y = zeros(1, length(t));
for i = 1:length(t)
    x(:,i+1) = A*x(:,i) + B*u(i);
    y(i) = C*x(:,i) + D*u(i);
end

figure('Name','Realistic desired velocity - Result of manual state space simulation')
plot(t, y, '-o')
hold on
plot(t, jsim, '-x')
plot(t, jreal)
legend('manual','lsim','python')
