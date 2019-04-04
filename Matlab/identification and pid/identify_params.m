% Identification of LTI models for x, y, z, yaw motion.
clear variables
close all
clc
fprintf('============ Start identification ============== \n')


%% Settings & Execution
options.figures = false;
options.prints = false;

% ----------------------------------------------------------------- 
% SYNTAX: 
%   model = identify("data/data_mat_file",'axis','axis symbol',Ts,f0,Fc,options);
% -----------------------------------------------------------------
xmodel = identify("data/angle_identification_x","x","x",0.02,0.53,0.6,options);
% xmodel_slow = identify("data/identification_x_cut","x","x",0.02,0.53,0.6,options);
ymodel = identify("data/angle_identification_y","y","y",0.02,0.53,0.6,options);
zmodel = identify("data/vel_identification_z","z","z",0.02,0.3,1.,options);
yawmodel = identify("data/vel_identification_yaw_preprocessed","yaw",char(952),0.02,0.3,1.,options);

% IMPORTANT NOTE: cutoff freq for x and y is based on crossover frequency (iteratively).
%       For z, no crossover (DC gain below 0 dB) --> visually (trial and
%       error. Look when oscillations that can't be fitted disappear but
%       information that CAN be fitted does not disappear.
%
% SECOND NOTE: Fc (cutoff for continuous butterworth LPF for inverting
%       velocity model) is a design parameter. Chosen such that flat part 
%       at high frequencies is +- 0 dB. 


fprintf('\n=========== Identification finished ============ \n')


%% ========================================================================
%                                Main function
%  ========================================================================

function model = identify(data_file, ax, axplot, Ts, f0, Fc, options)
% IDENTIFY - identifies LTI parameters for the drone model based on
% the data contained in the specified data file.
%
% Syntax:  data = identify(data_file)
%
% Input:
%   xdata_file - string representing data file name to perform parameter
%              identification on for the x-direction.
%   ydata_file -  y-direction
%   zdata_file - 
%
%
%
% Output:
%   model - struct with fields
%       tf_pos
%       tf_vel
%       ss_vel_invLPF
%       params - struct with fields
%           a_i, b_i (model parameters)
%       ...
%
% Example: 
%    xmodel = identify("data/angle_identification_x","x",0.02,0.53,0.6,options)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: data_file (input)
%
% Author: Rian Beck, Mathias Bos
% Website: https://github.com/BosMathias/BosRepo
% 2018-2019;

fprintf(strcat("\n----------------- ",axplot, ' direction ------------------\n'))


%% Load requested data file
data = load(data_file);


%% Extract some signals from the data

% Cutoff useful data
% first_index = find(input~=0, 1);
data.input = data.input(1:end-100)';
data.output = eval(strcat('data.output_', ax));
data.output = data.output(1:end-100)'-data.output(1);
input  = data.input;
output = data.output;

% Time & Frequency stuff
N = numel(output);
fs = 1/Ts;
t = (0:N-1)'*Ts;
f = (0:N-1)'*(fs/N);
data.Ts = Ts;
data.fs = fs;
data.t = t;
data.f = f;


% Differentiation of position
velocity = gradient(output)/Ts;
data.velocity = velocity;

if options.figures
    figure('Name','Measurement Data')
    subplot(211), plot(t, output, t, input), title(strcat(axplot,' position')), xlabel('time [s]'), ylabel('position [m]'), legend('output','input')
    subplot(212), plot(t, velocity), title(strcat(axplot,' velocity')), xlabel('time [s]'), ylabel('velocity [m/s]')
    
end

% Empirical frequency response
vel_fft = fft(velocity);
input_fft = fft(input);
data.FRF_emp = vel_fft./input_fft;


%% Cutoff frequency for Butterworth filtering
fc = 5*f0; % cutoff frequency (chosen)
fcn = fc/(fs/2); % normalized cutoff frequency (as butter() accepts)


%% Filtering of the in- and output data using Butterworth filter
if or(ax == "z", ax == "yaw")
    nb = 2;
else
    nb = 3;
end

[B, A] = butter(nb,fcn);
% input filtering
input_filt = filter(B,A,input);
% output filtering
velocity_filt = filter(B,A,velocity);

data.input_filt = input_filt;
data.velocity_filt = velocity_filt;

if options.figures
    figure('Name','filtered input')
    
    plot(t,input,t,input_filt)
    title('Input filtered')
    
    figure('Name','filtered output measurement')
    
    plot(t, velocity, t, velocity_filt)
    title(strcat('v_{',axplot,',filt}'))

end


%% Fitting parameters
if or(ax == "z", ax == "yaw")
    [params, tf_vel, data] = fit_1st_order(data, axplot, Ts, options);
else
    [params, tf_vel, data] = fit_2nd_order(data, axplot, Ts, options);
end

%% Integrating velocity models

[tf_pos, data] = integrate(tf_vel.cont, data, axplot, options);

%% Invert velocity model + LPF, state space for feedforward control

ss_vel_invLPF = invert_LPF_ss(tf_vel, ax, data, Fc, options);


%% Return results
model.params = params;
model.tf_vel = tf_vel;
model.tf_pos = tf_pos;
model.ss_vel_invLPF = ss_vel_invLPF;

end

%% ========================================================================
%                              Helper functions 
%  ========================================================================

function [params, transff, data] = fit_1st_order(data, axplot, Ts, options)
% id_1st_order - calculates transfer function parameters for 1st order 
% transfer function based on least squares fit for supplied in- and
% outputs.
%
% 1st order strictly proper minimum phase transfer function:
%
%                  b0
%      HVJ(z) = --------
%                z + a0
%
%
%   Inputs:
%       velocity_filt - Filtered velocity data (array)
%       input_filt - Filtered input data (array)
%
%   Output:
%       params - 1st order transfer function parameters (struct)
%           params.a = [1, a0]
%           params.b = [b0]
%           params.f0 - crossover frequency of the continuous time system
%       transff - 1st order discrete time & continuous time transfer
%                 functions
%           transf.discr - discrete time
%           transf.cont - continuous time


%% data
t = data.t;
f = data.f;

velocity = data.velocity;
velocity_filt = data.velocity_filt;
input         = data.input;
input_filt    = data.input_filt;


%% Fitting parameters (discrete time)
x = velocity_filt(2:end);
Phi = [-velocity_filt(1:end-1), input_filt(1:end-1)];
theta_filt = Phi\x;

params.b = theta_filt(2);
params.a = [1, theta_filt(1)];

transff.discr = tf(params.b, params.a, Ts);

FRF = squeeze(freqresp(transff.discr,2*pi*f));
data.FRF_vel = FRF;

if options.prints
   fprintf(strcat("\n* Discrete time velocity transfer function ",axplot,' direction:\n'))
   display(transff.discr) 
end

if options.figures
    

    figure('Name','1st - filtered - strictly proper - Minimum Phase: Freq Response')
    subplot(2,1,1)
    semilogx(f, 20*log10(abs(FRF)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('|FRF|  [m]')
    axis tight
    subplot(2,1,2)
    semilogx(f, 180/pi*unwrap(angle(FRF)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('\phi(FRF)  [^\circ]')

    x = lsim(transff.discr,input,t);

    figure('Name','1st - filtered - strictly proper - Minimum Phase: Simulation')
    subplot(211)
    hold on
    plot(t, velocity)
    plot(t, velocity_filt)
    plot(t, x)
    legend(strcat('v_{',axplot,',meas}'), strcat('v_{',axplot,',filt}'), strcat('v_{',axplot,',sim}'))
    title('1st - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
    xlabel('Time [s]')
    axis tight
    ylabel('Velocity [m/s]')
    subplot(212)
    plot(t,velocity - x)
    title('Difference between simulation and measurement')
    legend(strcat('v_{',axplot,',meas}-v_{',axplot,',sim}'))
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    axis tight

    figure('Name','1st - filtered - strictly proper - Minimum Phase: Pole Zero Map')
    pzmap(transff.discr)
    
end


%% Continuous time system
transff.cont = d2c(transff.discr,'matched');
% transff.cont = d2c(transff.discr,'tustin');
FRFc = squeeze(freqresp(transff.cont,2*pi*f));
data.FRFc_vel = FRFc;

if options.prints
   fprintf(strcat("\n* Continuous time velocity transfer function ",axplot,' direction:\n'))
   display(transff.cont) 
end

if options.figures
    figure('Name','Continuous - 1st - filtered - strictly proper - Minimum Phase: Freq Response')
    subplot(2,1,1)
    semilogx(f, 20*log10(abs(FRFc)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('|FRFc|  [m]')
    axis tight
    subplot(2,1,2)
    semilogx(f, 180/pi*unwrap(angle(FRFc)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('\phi(FRFc)  [^\circ]')

    xc = lsim(transff.cont,input,t);

    figure('Name','Continuous - 1st - filtered - strictly proper - Minimum Phase: Simulation')
    subplot(211)
    hold on
    plot(t, velocity,'g')
    plot(t, velocity_filt)
    plot(t,xc)
    legend(strcat('v_{',axplot,',meas}'), strcat('v_{',axplot,',filt}'), strcat('v_{',axplot,',sim}'))
    title('1st - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
    xlabel('Time [s]')
    axis tight
    ylabel('Velocity [m/s]')
    subplot(212)
    plot(t,velocity - xc)
    title('Difference between simulation and measurement')
    legend(strcat('v_{',axplot,',meas}-v_{',axplot,',sim}'))
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    axis tight

    figure('Name','Continuous - 1st - filtered - strictly proper - Minimum Phase: Pole Zero Map')
    pzmap(transff.cont)

end


%% Find crossover frequency of best fit
[~, index] = min(abs(20*log10(abs(FRFc(1:end-100)))));
f0 = f(index);
params.f0 = f0;

if options.prints 
    fprintf(strcat('Crossover - f0',axplot,': %d \n'), f0); 
end 


end

% -------------------------------------------------------------------------

function [params, transff, data] = fit_2nd_order(data, axplot, Ts, options)
% id_2nd_order - calculates transfer function parameters for 2nd order 
% transfer function based on least squares fit for supplied in- and
% outputs.
%
% 1st order strictly proper minimum phase transfer function:
%
%                       b0
%      HVJ(z) = -----------------
%                z^2 + a1*z + a0
%
%
%   Inputs:
%       velocity_filt - Filtered velocity data (array)
%       input_filt - Filtered input data (array)
%
%   Output:
%       params - 1st order transfer function parameters (struct)
%           params.a = [1, a1, a0]
%           params.b = [b0]
%       transff - 2nd order discrete time & continuous time transfer
%                 functions
%           transf.discr - discrete time
%           transf.cont - continuous time


%% data
t = data.t;
f = data.f;

velocity = data.velocity;
velocity_filt = data.velocity_filt;
input         = data.input;
input_filt    = data.input_filt;

%% Fitting parameters (discrete time)
x = velocity_filt(3:end);
Phi = [-velocity_filt(2:end-1), -velocity_filt(1:end-2), input_filt(1:end-2)];
theta_filt = Phi\x;

params.b = theta_filt(3);
params.a = [1, theta_filt(1) theta_filt(2)];

transff.discr = tf(params.b, params.a, Ts);

FRF = squeeze(freqresp(transff.discr,2*pi*f));
data.FRF_vel = FRF;

if options.prints
   fprintf(strcat("\n* Discrete time velocity transfer function ",axplot,' direction:\n'))
   display(transff.discr) 
end

if options.figures
    

    figure('Name','2nd - filtered - strictly proper - Minimum Phase: Freq Response')
    subplot(2,1,1)
    semilogx(f, 20*log10(abs(FRF)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('|FRF|  [m]')
    axis tight
    subplot(2,1,2)
    semilogx(f, 180/pi*unwrap(angle(FRF)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('\phi(FRF)  [^\circ]')

    x = lsim(transff.discr,input,t);

    figure('Name','2nd - filtered - strictly proper - Minimum Phase: Simulation')
    subplot(211)
    hold on
    plot(t, velocity)
    plot(t, velocity_filt)
    plot(t, x)
    legend(strcat('v_{',axplot,',meas}'), strcat('v_{',axplot,',filt}'), strcat('v_{',axplot,',sim}'))
    title('2nd - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
    xlabel('Time [s]')
    axis tight
    ylabel('Velocity [m/s]')
    subplot(212)
    plot(t,velocity - x)
    title('Difference between simulation and measurement')
    legend(strcat('v_{',axplot,',meas}-v_{',axplot,',sim}'))
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    axis tight

    figure('Name','2nd - filtered - strictly proper - Minimum Phase: Pole Zero Map')
    pzmap(transff.discr)
    
end


%% Continuous time system
transff.cont = d2c(transff.discr,'matched');
% transff.cont = d2c(transff.discr,'tustin');

FRFc = squeeze(freqresp(transff.cont,2*pi*f));
data.FRFc_vel = FRFc;

if options.prints
   fprintf(strcat("* Continuous time velocity transfer function ",axplot,' direction:\n'))
   display(transff.cont) 
end

if options.figures
    figure('Name','Continuous - 2nd - filtered - strictly proper - Minimum Phase: Freq Response')
    subplot(2,1,1)
    semilogx(f, 20*log10(abs(FRFc)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('|FRFc|  [m]')
    axis tight
    subplot(2,1,2)
    semilogx(f, 180/pi*unwrap(angle(FRFc)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('\phi(FRFc)  [^\circ]')

    xc = lsim(transff.cont,input,t);

    figure('Name','Continuous - 2nd - filtered - strictly proper - Minimum Phase: Simulation')
    subplot(211)
    hold on
    plot(t, velocity,'g')
    plot(t, velocity_filt)
    plot(t,xc)
    legend(strcat('v_{',axplot,',meas}'), strcat('v_{',axplot,',filt}'), strcat('v_{',axplot,',sim}'))
    title('2nd - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
    xlabel('Time [s]')
    axis tight
    ylabel('Velocity [m/s]')
    subplot(212)
    plot(t,velocity - xc)
    title('Difference between simulation and measurement')
    legend(strcat('v_{',axplot,',meas}-v_{',axplot,',sim}'))
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    axis tight

    figure('Name','Continuous - 2nd - filtered - strictly proper - Minimum Phase: Pole Zero Map')
    pzmap(transff.cont)

end


%% Find crossover frequency of best fit
[~, index] = min(abs(20*log10(abs(FRFc(1:end-100)))));
f0 = f(index);
params.f0 = f0;

if options.prints 
    fprintf(strcat('Crossover - f0',axplot,': %d \n'), f0); 
end 


end

% -------------------------------------------------------------------------

function [tf_pos, data] = integrate(tf_vel, data, axplot, options)
s = tf('s');
tf_pos = 1/s*tf_vel;

input = data.input;
output = data.output;
t = data.t;
f = data.f;

FRF = squeeze(freqresp(tf_pos,2*pi*f));
data.FRFc_pos = FRF;

if options.prints
   fprintf(strcat("\n* Continuous time position transfer function ",axplot,' direction:\n'))
   display(tf_pos) 
end

if options.figures


    figure('Name','Integrated, filtered, strictly proper - Freq. Resp.'), subplot(211)
    semilogx(f, 20*log10(abs(FRF)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('|FRF|  [m]')
    subplot(212),semilogx(f, 180/pi*unwrap(angle(FRF)))
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('\phi(FRF)  [^\circ]')

    x = lsim(tf_pos,input,t);

    figure('Name','Integrated, filtered, strictly proper - Simulation')
    subplot(211)
    hold on
    plot(t, output,'g')
    plot(t, x)
    title('Integrated, filtered, strictly proper - Simulation VS Measurement')
    legend(strcat('pos_{',axplot,',meas}'), strcat('pos_{',axplot,',sim}'))
    xlabel('Time [s]')
    ylabel('Displacement [m]')
    axis tight
    subplot(212)
    plot(t,output - x)
    title('Difference between simulation and measurement')
    legend(strcat('pos_{',axplot,',meas} - pos_{',axplot,',sim}'))
    xlabel('Time [s]')
    ylabel('Displacement [m]')
    axis tight

    figure('Name','Integrated, filtered, strictly proper - Pole Zero Map')
    pzmap(tf_pos)
end

end

% -------------------------------------------------------------------------

function [ss_vel_invLPF, data] = invert_LPF_ss(tf_vel, ax, data, Fc, options)
t = data.t;
f = data.f;
dt = data.Ts;
input = data.input;

sys_c = tf_vel.cont;

FRF_emp = data.FRF_emp;
FRFc = data.FRFc_vel;

% Difference
FRF_diff = (FRF_emp-FRFc)./FRFc;


%% Low pass filtering the inverse system ( = multiplying the regular system with inverse LPF)
if or(ax == "z", ax == "yaw")
    nb = 1;
else
    nb = 2;
end


[Bpre, Apre] = butter(nb, Fc*2*pi, 's'); % continuous time!

filt = tf(Bpre,Apre);
FRF_LPF = squeeze(freqresp(filt,2*pi*f));


LPF = tf(Bpre,Apre);

sys_LPF = sys_c/LPF;


if options.figures
    figure('Name','Low Pass Filter (Butterworth)')
    bode(LPF)

    figure('Name','Butterworth filtered continuous time system')
    bode(sys_c)
    hold on
    bode(sys_LPF)
    legend('Identified system before filtering','Filtered system')

    figure('Name','Difference Empirical - Continuous VS inverse filter')
    subplot(2,1,1)
    semilogx(f, 20*log10(abs(FRF_diff)), 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',1.5)
    hold on
    semilogx(f, 20*log10(abs(FRF_LPF.^(-1))), 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
    grid on 
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('H(f)|  [m]')
    legend('FRF_{diff}', 'LPF')
    axis tight
    subplot(2,1,2)
    semilogx(f, 180/pi*unwrap(angle(FRF_diff)), 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',1.5)
    hold on
    semilogx(f, 180/pi*unwrap(angle(FRF_LPF.^(-1))), 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5)
    grid on
    xlim([f(1) f(end)])
    xlabel('f  [Hz]')
    ylabel('\phi(H(f))  [^\circ]')
    legend('FRF_{diff}', 'LPF')
end


%% Discretize filtered system to 100Hz

sys_dLPF = c2d(sys_LPF,0.01,'tustin');

if options.figures
    figure('Name','Butterworth filtered, discretized (100Hz) system: Freq resp')
    bode(sys_dLPF)

    figure('Name', 'Butterworth filtered, discretized (100Hz) system: Pole Zero Map')
    pzmap(sys_dLPF)
end

%% Discretize filtered system
sys_dLPF = c2d(sys_LPF,0.01,'tustin');


%% State space representation of the filtered system and simulation

[b_dLPFi, a_dLPFi] = tfdata(sys_dLPF^(-1), 'v');
[A_dLPFi, B_dLPFi, C_dLPFi, D_dLPFi] = tf2ss(b_dLPFi, a_dLPFi);
ss_vel_invLPF = ss(A_dLPFi,B_dLPFi,C_dLPFi,D_dLPFi,0.01);
    
if options.figures
    % Simulate on realistic desired speed signal: interpolated simulation result
    dt100Hz = .01;
    t100Hz = (0:dt100Hz:(length(input)-1)*dt)';
    % - lsim simulation    
    x_50Hz = lsim(tf_vel.discr,input,t);
    x_100Hz = interp1(t,x_50Hz,t100Hz);
    sim_ss = lsim(ss_vel_invLPF, x_100Hz, t100Hz);
    figure('Name', 'Realistic desired velocity - Result of lsim')
    plot(t100Hz, sim_ss)

    % - manual simulation
    if or(ax == "z", ax == "yaw")
        xsim = zeros(1, length(t100Hz));
        xstep = zeros(1, length(t100Hz));
    else
        xsim = zeros(2, length(t100Hz));
        xstep = zeros(2, length(t100Hz));
    end
    ysim = zeros(1, length(t100Hz));
    for i = 1:length(t100Hz)
        xsim(:,i+1) = A_dLPFi*xsim(:,i) + B_dLPFi*x_100Hz(i);
        ysim(i) = C_dLPFi*xsim(:,i) + D_dLPFi*x_100Hz(i);
    end

    figure('Name','Realistic desired velocity - Result of manual state space simulation')
    plot(t100Hz, ysim)

    % step response:
    vstep = 0.2*ones(length(t100Hz),1);
    
    ystep = zeros(1, length(t100Hz));
    for i = 1:length(t100Hz)
        xstep(:,i+1) = A_dLPFi*xstep(:,i) + B_dLPFi*vstep(i);
        ystep(i) = C_dLPFi*xstep(:,i) + D_dLPFi*vstep(i);
    end

    figure('Name','Result of manual state space simulation - Step input')
    plot(t100Hz, ystep, t100Hz, vstep)
    legend('step response','step input')
end


end
