% PD controller interpretation

% COLORS FOR FIGURES:
% Blue
% 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5
% Yellow
% 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',2.5
% Red
% 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5

clear variables
close all
clc


%% Identify models
run identify_params


%% Calculate PD controller parameters
fprintf('\n== Start PD controller parameter calculation ==\n')
options.figures = false;
options.prints = false;

% Desired phase margin
PM_des = 45;

% Function calls
fprintf('\n----------------- x direction -----------------\n')
[xsys_cl,xPDparams] = PD_design(xmodel, PM_des, options);
fprintf('\n----------------- y direction -----------------\n')
[ysys_cl,yPDparams] = PD_design(ymodel, PM_des, options);

% -------------------------------------------------------------------------
% fprintf('\n----------------- z direction -----------------\n')
% [zsys_cl,zPDparams] = PD_design(zmodel, PM_des, options);
% NOTE - z-direction proportional controller sufficient. 
%        This procedure doesn't work for z.


fprintf('\n===== PD controller design finished ====\n')


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

%% PD without Kp:
% D(s) = Kp * (1 + Td*s)
% Dk(s) = (1 + Td*s), D(s) = Kp*Dk(s)
Dk = (1 + Td*s);

%% Determine Kp & Kd
mag = abs((evalfr(G*Dk,1j*w_c)));
Kp = 1/(mag);
Kd = Kp*Td;

%% Resulting PD-controller
D = Kp*Dk;
PDparams.Td = Td;
PDparams.Kp = Kp;
PDparams.Kd = Kd;

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
    figure('Name', 'PD controller bode')
    bode(D)
    
    figure('Name','Margins of closed loop system for varying control parameter Kp')
    hold on
    margin(G)
    margin(sys_ol);
    % Some dirty manipulating to choose color and line thickness:
    bodeplot(G,'b');
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',[0.3010, 0.7450, 0.9330]);
    bodeplot(sys_ol,'b');
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',[0.6350, 0.0780, 0.1840])
    
    set(findall(gcf,'type','line'),'linewidth',2)
    
    h = flipud(findobj(gcf,'type','Axes'));
    hl1 = flipud(findobj(h(1),'type','Line'));
    hl2 = flipud(findobj(h(2),'type','Line'));
 
    legend(h(1),hl1(3:4),'Uncompensated','PD Compensated')
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
    figure('Name','Pzmap of closed loop system for varying control parameter Kp')
    pzplot(sys_cl)
end


end

