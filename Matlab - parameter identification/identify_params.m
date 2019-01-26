clear variables
close all
clc
fprintf('-- Start identification -- \n')


%%
options.figures = true;
options.prints = true;


% xmodel = identify("data/angle_identification_x",'x',0.02,0.5,options);
% ymodel = identify("data/angle_identification_y",'y',0.02,0.5,options);
zmodel = identify("data/vel_identification_z_short",'z',0.02,0.3,options);

% IMPORTANT NOTE: cutoff freq for x and y is based on crossover frequency (iteratively).
%       For z, no crossover (DC gain below 0 dB) --> visually (trial and
%       error. Look when vibrations that can't be fitted disappear but
%       information that CAN be fitted does not.


fprintf('\n-- Identification finished -- \n')


%% Main function


function model = identify(data_file, ax, Ts, f0, options)
% IDENTIFY - identifies LTI parameters for the drone model based on
% the data contained in the specified data file.
%
% Syntax:  data = identify_params(data_file)
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
%       ss_pos
%       ss_vel
%       params - struct with fields
%           a_i, b_i (model parameters)
%       ...
%
% Example: 
%    x_data = identify_params("angle_identification_x")
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: data_file (input)
%
% Author: Rian Beck, Mathias Bos
% Website: https://github.com/BosMathias/BosRepo
% 2018-2019;

fprintf(strcat("\n========================== ",ax, ' direction ==========================\n'))


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
    subplot(211), plot(t, output, t, input), title(strcat(ax,' position')), xlabel('time [s]'), ylabel('position [m]'), legend('output','input')
    subplot(212), plot(t, velocity), title(strcat(ax,' velocity')), xlabel('time [s]'), ylabel('velocity [m/s]')
    
end


%% Cutoff frequency for Butterworth filtering
fc = 5*f0; % cutoff frequency (chosen)
fcn = fc/(fs/2); % normalized cutoff frequency (as butter() accepts)


%% Filtering of the in- and output data using Butterworth filter
if ax == 'z'
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
    title(strcat('v_{',ax,',filt}'))

end


%% Fitting parameters
if ax == 'z'
    [params, transff] = fit_1st_order(data, ax, Ts, options);
else
    [params, transff] = fit_2nd_order(data, ax, Ts, options);
end

%% Integrating velocity models



%% Invert velocity model + LPF, state space for feedforward control




%% Return results
model.params = params;
model.transff = transff;
% model.ss_invLPF = 

end

% -------------------------------------------------------------------------
%% Helper functions


function [params, transff] = fit_1st_order(data, ax, Ts, options)
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

if options.prints
   fprintf(strcat("\nDiscrete time transfer function ",ax,' direction:\n'))
   fprintf('--------------------------------------------')
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

    figure('Name','2nd - filtered - strictly proper - Minimum Phase: Simulation')
    subplot(211)
    hold on
    plot(t, velocity)
    plot(t, velocity_filt)
    plot(t, x)
    legend(strcat('v_{',ax,',meas}'), strcat('v_{',ax,',filt}'), strcat('v_{',ax,',sim}'))
    title('1st - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
    xlabel('Time [s]')
    axis tight
    ylabel('Velocity [m/s]')
    subplot(212)
    plot(t,velocity - x)
    title('Difference between simulation and measurement')
    legend(strcat('v_{',ax,',meas}-v_{',ax,',sim}'))
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    axis tight

    figure('Name','1st - filtered - strictly proper - Minimum Phase: Pole Zero Map')
    pzmap(transff.discr)
    
end


%% Continuous time system
transff.cont = d2c(transff.discr,'matched');
FRFc = squeeze(freqresp(transff.cont,2*pi*f));

if options.prints
   fprintf(strcat("Continuous time transfer function ",ax,' direction:\n'))
   fprintf('----------------------------------------------')
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
    legend(strcat('v_{',ax,',meas}'), strcat('v_{',ax,',filt}'), strcat('v_{',ax,',sim}'))
    title('1st - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
    xlabel('Time [s]')
    axis tight
    ylabel('Velocity [m/s]')
    subplot(212)
    plot(t,velocity - xc)
    title('Difference between simulation and measurement')
    legend(strcat('v_{',ax,',meas}-v_{',ax,',sim}'))
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
    fprintf(strcat('Crossover - f0',ax,': %d \n'), f0); 
end 



end


function [params, transff] = fit_2nd_order(data, ax, Ts, options)
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

if options.prints
   fprintf(strcat("\nDiscrete time transfer function ",ax,' direction:\n'))
   fprintf('--------------------------------------------')
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
    legend(strcat('v_{',ax,',meas}'), strcat('v_{',ax,',filt}'), strcat('v_{',ax,',sim}'))
    title('2nd - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
    xlabel('Time [s]')
    axis tight
    ylabel('Velocity [m/s]')
    subplot(212)
    plot(t,velocity - x)
    title('Difference between simulation and measurement')
    legend(strcat('v_{',ax,',meas}-v_{',ax,',sim}'))
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    axis tight

    figure('Name','2nd - filtered - strictly proper - Minimum Phase: Pole Zero Map')
    pzmap(transff.discr)
    
end


%% Continuous time system
transff.cont = d2c(transff.discr,'matched');
FRFc = squeeze(freqresp(transff.cont,2*pi*f));

if options.prints
   fprintf(strcat("Continuous time transfer function ",ax,' direction:\n'))
   fprintf('----------------------------------------------')
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
    legend(strcat('v_{',ax,',meas}'), strcat('v_{',ax,',filt}'), strcat('v_{',ax,',sim}'))
    title('2nd - filtered - strictly proper - Minimum Phase: Simulation vs Measurement')
    xlabel('Time [s]')
    axis tight
    ylabel('Velocity [m/s]')
    subplot(212)
    plot(t,velocity - xc)
    title('Difference between simulation and measurement')
    legend(strcat('v_{',ax,',meas}-v_{',ax,',sim}'))
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
    fprintf(strcat('Crossover - f0',ax,': %d \n'), f0); 
end 


end

