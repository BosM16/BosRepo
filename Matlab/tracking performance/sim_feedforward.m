% Simulate 8 tracking

close all
clear variables
clc

run("identification and pid/identify_params")
run("tracking performance/tracking_performance")

%%
fs = 100;
f0 = 0.5;
fc = f0; % cutoff frequency (chosen)
fcn = fc/(fs/2); % 
nb = 2;
[B, A] = butter(nb,fcn);

xref = data.drawn_path_x - data.drawn_path_x(1);
xref = [xref(1)*ones(1,200) xref];% xref(end)*ones(1,200)];
t    = 0:0.01:(length(xref)-1)*0.01;
xref_filt = filtfilt(B,A,xref);
% xref_filt = xref_filt - xref_filt(1);


vref = gradient(xref_filt)./0.01;
% vref = vref - vref(1);

figure
subplot(211)
hold on
plot(xref)
plot(xref_filt)
legend('ref','filtered ref')
subplot(212)
plot(vref)

jsim = lsim(xmodel.ss_vel_invLPF,vref,t);
jsim_clip = max(min(jsim,1),-1);
figure
hold on
plot(t,jsim)
plot(t,jsim_clip)
plot(t,vref)
legend('jsim','jsim clipped','vref')


xsim = lsim(xmodel.tf_pos,jsim,t);
xsim_clip = lsim(xmodel.tf_pos,jsim_clip,t);

figure
hold on
plot(t,xsim)
plot(t,xsim_clip)
plot(t,xref)
legend('sim','sim clipped','ref')
