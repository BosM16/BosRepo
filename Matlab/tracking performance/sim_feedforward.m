% Simulate 8 tracking

% close all
% clear variables
clc

run("identification and pid/identify_params")
run("tracking performance/tracking_performance")

%%
fs = 100;
f0 = 0.5;
fc = f0; % cutoff frequency (chosen)
fcn = fc/(fs/2); % 
nb = 3;
[B, A] = butter(nb,fcn);

xref = data.drawn_path_x - data.drawn_path_x(1);
xrefcut = xref(51:end);

% Padding
% xref = [xref(1)*ones(1,50) xref xref(end)*ones(1,50)];

% filtfilting
% xref_filt = filtfilt(B,A,xref);
% xref_filt = xref_filt - xref_filt(1);


vref = gradient(xrefcut)./0.01;

% revert
vref = fliplr(vref);
% linear padding
padlen = 50;

dx = 3/2*vref(end)-2*vref(end-1)+1/2*vref(end-2);
range = dx:dx:padlen*dx;
xpad = vref(end) + range;
vref = [vref xpad]; 

vref = filter(B,A,vref);

vref = fliplr(vref);
vref = vref(1:end-50);

xref = xref(1:end-50);
t    = 0:0.01:(length(xref)-1)*0.01;


% inverse model
jsimff = lsim(xmodel.ss_vel_invLPF,vref,t);
jsim_clip = max(min(jsimff,1),-1);

% simulate pos
xsimff = lsim(xmodel.tf_pos,jsimff,t);
xsim_clip = lsim(xmodel.tf_pos,jsim_clip,t);


%%

figure
hold on
plot(t,jsimff)
plot(t,jsim_clip)
plot(t,vref)
legend('jsim','jsim clipped','vref')

figure
hold on
plot(t,xsimff)
plot(t,xsim_clip)
plot(t,xref)
legend('sim','sim clipped','ref')
