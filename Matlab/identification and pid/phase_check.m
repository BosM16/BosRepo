% Check if the phase shift in resulting traj is gone.
% close all
% clear variables
clc

% run identify_params

%% load data
data = load('/home/mathias/Documents/BosRepo/Matlab/identification and pid/data/tracking_fb_path_check.mat');

%%

vely = data.fb_vel_y;
posy = data.fb_path_y(1:end-1);
vyFD = gradient(posy)/0.01;


figure
hold on
plot(vely)
plot(vyFD)
plot(posy)
legend('vel','velFD','pos')


%%

% input_y = data.input_y(length(data.input_y)-length(data.drawn_path_x)+1:end);
% vel_in_y = data.vel_in_y(length(data.vel_in_y)-length(data.drawn_path_x)+1:end);
input_y = data.input_y;
vel_in_y = data.vel_in_y;
data.drawn_path_x = data.drawn_path_x(1:end-52);
data.drawn_path_x = [data.drawn_path_x(1)*ones(1,40) data.drawn_path_x];

x_drawn = -(data.drawn_path_x - data.drawn_path_x(1));
vxFD = gradient(x_drawn)/0.01;

t = 0:0.01:(length(input_y)-1)*0.01;
ysim = lsim(ymodel.tf_pos,input_y,t);
vysim = lsim(ymodel.tf_vel.cont,input_y,t);

jvXD = lsim(ymodel.ss_vel_invLPF,vxFD,t);
jsim = lsim(ymodel.ss_vel_invLPF,vel_in_y,t);

figure
hold on
plot(t,jsim,'-o')
plot(t,input_y,'-x')
plot(t,jvXD,'--')
legend('input sim','input python','from vxFD')


figure
hold on
plot(t,vel_in_y)
plot(t,vysim)
plot(t,vxFD)
legend('filtered vel','fwd sim','FD from drawn')

figure
hold on
plot(t,x_drawn)
plot(t,ysim)
legend('drawn','sim')


%%
Bpy = [6.45184913e-6 ];
Apy = [1. -2.92460624 2.85202782 -0.92736997];
