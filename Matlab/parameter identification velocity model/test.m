clear all
close all
%%

load angle_identification_y

t = 0:0.02:(length(input)-1)/50;

figure
hold on
plot(t, input)
plot(t, output_x)
plot(t, output_y)
plot(t, output_z)
legend('input','x','y','z')