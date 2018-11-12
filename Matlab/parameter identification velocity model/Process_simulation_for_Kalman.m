close all
clear all
%% load signals
x = load('XSignals_50Hz');
y = load('YSignals_50hz');

%% Interpolate input to have 100Hz input
dt50Hz = 0.02;
dt100Hz = 0.01;
% all signals in x and y are sampled at 50Hz
time50Hz = (0:dt50Hz:(length(x.input)-1)*dt50Hz)';
time100Hz = (0:dt100Hz:(length(x.input)-1)*dt50Hz)';

% create input signal that is sampled at 100Hz
x.input100Hz = interp1(time50Hz, x.input, time100Hz);
y.input100Hz = interp1(time50Hz, y.input, time100Hz);

%% Correct interpolated values in between jumps.
ignore_nextx = false;
ignore_nexty = false;
for i=(2:length(time100Hz))
    
    if ignore_nextx
        ignore_nextx = false;
    elseif abs(x.input100Hz(i)-x.input100Hz(i-1)) > 0.1;
        x.input100Hz(i) = x.input100Hz(i-1);
        ignore_nextx = true;
        
    end
    if ignore_nexty
        ignore_nexty = false;
    elseif abs(y.input100Hz(i)-y.input100Hz(i-1)) > 0.1;
        y.input100Hz(i) = y.input100Hz(i-1);
        ignore_nexty = true;
    end
end
%% plot the result
figure()
subplot(211),plot(time50Hz, x.input,'*', time100Hz, x.input100Hz, 'o')
subplot(212),plot(time50Hz, x.input,'*', time100Hz, x.input100Hz, 'o')

%% save the result
save('XYSignals','x','y')
