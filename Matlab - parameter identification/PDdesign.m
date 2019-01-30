% PD controller interpretation

clear variables
close all
clc


%% Identify models
run identify_params


%% Calculate PD controller parameters
fprintf('\n== Start PD controller parameter calculation ==\n')
options.figures = false;
options.prints = true;

% Desired phase margin
PM_des = 45;

% Function calls
fprintf('\n----------------- x direction -----------------\n')
[xsys_cl,xPDparams] = PD_design(xmodel, PM_des, options);
fprintf('\n----------------- y direction -----------------\n')
[ysys_cl,yPDparams] = PD_design(ymodel, PM_des, options);

% -------------------------------------------------------------------------
fprintf('\n----------------- z direction -----------------\n')
[zsys_cl,zPDparams] = PD_design(zmodel, PM_des, options);
% NOTE - z-direction proportional controller sufficient. 
%        This procedure doesn't work for z.


fprintf('\n===== Closed loop stability check finished ====\n')


%% ========================================================================
%                               Main function
%  ========================================================================

function [sys_cl,PDparams] = PD_design(model, PM, options)

G = model.tf_pos;

w = logspace(0,2,10^3);
s = tf('s');

[~,Gphase,~] = bode(G,w);
Gphase = squeeze(Gphase);

% figure('Name','Bode & margins of uncompensated system')
% margin(G);

%% Determine Td
%  Td chosen as function of PM.
%  Correction because not exactly 90deg is added.
corr = 6;
phi = -180 + PM - 90 + corr;
% crossover frequency
w_c = interp1(Gphase,w,phi);

Td = 10/w_c;

%% PD without Kx:
% D(s) = Kx * (1 + Td*s)
% Dk(s) = (1 + Td*s), D(s) = Kx*Dk(s)
Dk = (1 + Td*s);

%% Determine Kx & Kv
mag = abs((evalfr(G*Dk,1j*w_c)));
Kx = 1/(mag);
Kv = Kx*Td;

%% Resulting PD-controller
D = Kx*Dk;
PDparams.Td = Td;
PDparams.Kx = Kx;
PDparams.Kv = Kv;

% Display calculated parameters in command window
if options.prints
    fprintf('\nPD parameters\n')
    disp(PDparams)
end


%% Visualisation & Command window information

sys_ol = G*D;
sys_cl = G*D/(1+G*D);

[Gm,Pm,Wcg,Wcp] = margin(sys_ol);

if options.figures
    figure('Name','Margins of closed loop system for varying control parameter Kx')
    hold on
    margin(G)
    margin(sys_ol);
    legend('Uncompensated','PD Compensated')
end

% Display margins, poles in command window
if options.prints
    fprintf('Phase margin, Gain margin, Gain w_c, Phase w_c\n')
    disp([Pm,Gm,Wcg,Wcp])

    fprintf('\nPoles of closed loop system:\n')
    disp(pole(sys_cl))

end

% Plot as function of the varying parameters
if options.figures
    figure('Name','Pzmap of closed loop system for varying control parameter Kx')
    pzplot(sys_cl)
end


end

