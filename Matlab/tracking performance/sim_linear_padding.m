
dt = 0.01;
t = 0:dt:10;
tramp = dt:dt:1;
padlen = 50;
s = sin(t);
ramp = (3/2*s(end)-2*s(end-1)+1/2*s(end-2))/dt*(tramp);

s = [s s(end)+ramp];
t = 0:dt:(length(s)-1)*dt;

sfilt = filter(B,A,s);

sfiltfilt = filter(

figure
hold on
plot(t,s)
plot(t,sfilt)
plot(t-0.5,sfilt)