% State feedback controller interpretation

clear variables
close all
%% Identify models
run identify_params

%% Check stability of closed loop system
fprintf('\n-- Start closed loop stability check --')
% Control parameters
%   - x direction
xctrl_params.Kx = 0.3;
% xctrl_params.Td = 0.4;
% xctrl_params.Kv = xctrl_params.Kx * xctrl_params.Td;
%   - y direction
yctrl_params.Kx = 0.1;
yctrl_params.Td = 0.2;
yctrl_params.Kv = yctrl_params.Kx * yctrl_params.Td;
%   - z direction
zctrl_params.Kx = 0.1;
zctrl_params.Td = 0.2;
zctrl_params.Kv = zctrl_params.Kx * zctrl_params.Td;

% Function calls
xsys_cl = cl_stability(xmodel, xctrl_params);
% ysys_cl = cl_stability(ymodel, yctrl_params);
% zsys_cl = cl_stability(xmodel, xctrl_params);

fprintf('-- Closed loop stability check finished --\n')



%% ========================================================================
%                               Main function
%  ========================================================================

function sys_cl = cl_stability(model, ctrl_params)

Kx = ctrl_params.Kx;
% Td = ctrl_params.Td;
% Kv = ctrl_params.Kv;

H = [model.tf_pos; model.tf_vel.cont];
figure('Name','Bode of uncompensated system')
bode(H);

%% Determine Td
%  Td chosen such that 10/Td = w_c of position model.
[~,~,~,w_c] = margin(H(1));
Td = 10/w_c;


%% Closed loop stability for varying Kx
K = Kx*[1 Td];
% K = [Kx Kv];
sys_ol = H*K;
sys_cl = H*K*(eye(2)+K*H)^(-1);

[Gm11,Pm11,Wcg11,Wcp11] = margin(sys_ol(1,1));
[Gm12,Pm12,Wcg12,Wcp12] = margin(sys_ol(1,2));
[Gm21,Pm21,Wcg21,Wcp21] = margin(sys_ol(2,1));
[Gm22,Pm22,Wcg22,Wcp22] = margin(sys_ol(2,2));


figure
k=1;
for i = 1:2
    for j = 1:2
    subplot(2,2,k)
    margin(sys_ol(i,j));
    k=k+1;
    end
end

figure('Name','Margins of closed loop system for varying control parameter Kx')
hold on
for Kx = 0.01:3:10
    K = Kx*[1 Td];
    sys_ol = H*K;
    
    k=1;
    for i = 1:2
        for j = 1:2
    subplot(2,2,k)
    margin(sys_ol(i,j));
    k=k+1;
        end
    end
end
hold off

% Display margins in command window
Pm = [Pm11,Pm12,Pm21,Pm22];
Gm = [Gm11,Gm12,Gm21,Gm22];
Wcg = [Wcg11,Wcg12,Wcg21,Wcg22];
Wcp = [Wcp11,Wcp12,Wcp21,Wcp22];
fprintf('\nPhase margin:\n')
disp(Pm)
fprintf('\nGain margin:\n')
disp(Gm)

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
for Kx = 10^-4:1:10
    K = Kx*[1 Td];
    sys_cl = H*K*(1+H*K)^(-1);
    pzmap(sys_cl)
   
end
hold off

end