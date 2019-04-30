
fs = 100;
f0 = 10;
fc = f0; % cutoff frequency (chosen)
fcn = fc/(fs/2); % 
nb = 2;
[B, A] = butter(nb,fcn);


ramp = 0:100;
signal = [zeros(1,100) ramp];

ramp_filt = filter(B,A,signal);
ramp_filtfilt =filtfilt(B,A,signal);

figure
hold on
plot(signal)
plot(ramp_filt)
plot(ramp_filtfilt)
legend('ramp','filter','filtfilt')
