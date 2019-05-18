close all
% clear variables
%%
% run("identification and pid/identify_params")

data = load('data_new/gans_kaput.mat');

figure
hold on
plot(data.vel_in_y)
plot(-data.fb_vel_x)
legend('vel_in_x','fb_vel')

t = 0:0.01:0.01*(length(data.vel_in_y)-1);

% sim
jsim = lsim(ymodel.ss_vel_invLPF,data.vel_in_y,t);

figure
hold on 
plot(data.input_y,'-o')
plot(jsim,'-x')
legend('python','matlab')

xref = data.real_path_x;

xsim = lsim(xmodel.tf_pos,jsim,t);
figure
hold on
plot(t,xsim)
plot(t,-(xref-xref(1)))
legend('sim','ref')