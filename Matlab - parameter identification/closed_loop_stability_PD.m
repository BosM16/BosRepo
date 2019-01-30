% PD controller interpretation

clear variables
% close all


%% Identify models
run identify_params


%% Check stability of closed loop system
fprintf('\n-- Start closed loop stability check --\n')
% Control parameters
%   - x direction
xctrl_params.Kx = 3;
xctrl_params.Td = 10;
%   - y direction
yctrl_params.Kx = 0.1;
yctrl_params.Td = nan;
%   - z direction
zctrl_params.Kx = 0.1;
zctrl_params.Td = nan;

% Function calls
[xsys_cl,xctrl_params] = PD_design(xmodel, xctrl_params);
% [ysys_cl,yctrl_params] = PD_design(ymodel, yctrl_params);
% [zsys_cl,zctrl_params] = PD_design(zmodel, zctrl_params);

fprintf('-- Closed loop stability check finished --\n')


%% ========================================================================
%                               Main function
%  ========================================================================

function [sys_cl,ctrl_params] = PD_design(model, ctrl_params)
if isnan(ctrl_params.Kx)
   ll 
else
    Kx = ctrl_params.Kx;
end
    % Td = ctrl_params.Td;
% Kv = ctrl_params.Kv;
s = tf('s');

H = model.tf_pos;
figure('Name','Bode of uncompensated system')
margin(H);

%% Determine Td
% If Td is not provided, calculate it here.
%  Td chosen such that 10/Td = w_c of position model.
fprintf('\nIgnore the warning below: uncompensated system is unstable.\n')
[~,~,~,w_c] = margin(H*Kx);  % Warning that it's unstable: of course, it's uncompensated.
if isnan(ctrl_params.Td)
    Td = 10/w_c;
    ctrl_params.Td = Td;
else
    Td = ctrl_params.Td;
end
Kv = Kx*Td;
ctrl_params.Kv = Kv;

%% Closed loop stability for varying Kx
% D = Kx*(1 + Td*s);
D = Kx + Kv*s;
% K = [Kx Kv];
sys_ol = H*D;
sys_cl = H*D/(1+H*D);

[Gm,Pm,Wcg,Wcp] = margin(sys_ol);


figure('Name','Margins of closed loop system for varying control parameter Kx')
hold on
margin(H)
lgnd = ["Uncompensated"];
margin(sys_ol);
lgnd = [lgnd ;"Selected parameters"];
for Kx = 0.01:0.5:2
    D = Kx*(1 + Td*s);
    sys_ol = H*D;
    margin(sys_ol);
    lgnd = [lgnd;'Kx = ' + string(Kx)]; %#ok<AGROW>
end
legend(lgnd)
hold off

% Display margins in command window
fprintf('Phase margin, Gain margin, Gain crossover, Phase crossover\n')
disp([Pm,Gm,Wcg,Wcp])

% figure('Name','Bode plot of closed loop system for selected control parameters')
% bode(sys_cl)

fprintf('\nPoles of closed loop system:\n')
disp(pole(sys_cl))

% Plot as function of the varying parameters
figure('Name','Pzmap of closed loop system for varying control parameter Kx')
pzplot(sys_cl)
a = findobj(gca,'type','line');
set(a(2),'markersize',10)
set(a(3),'markersize',10)
hold on
lgnd = ['sys_{cl}'];
for Kx = 0.01:0.6:3
    D = Kx*(1 + Td*s);
    sys_cl = H*D/(1+H*D);
    pzmap(sys_cl)
    lgnd = [lgnd;'Kx = ' + string(Kx)]; %#ok<AGROW>
end
legend(lgnd)

hold off

end