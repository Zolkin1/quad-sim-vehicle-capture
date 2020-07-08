%% Motor Control Simulation
close all;
clear all;

b = 0.05;
J = 0.01;
R = 5;
k = 0.006;

kp = 1;
kff = 1;

s = tf('s');
%motortf = k/((J*s + b)*R + k^2);
motortf = 2000/(s+20);
step(motortf);

Cf = kff;
C = kp;

Cf =tf(CF);
C = tf(C);

%Cf = 40 and C = 10 seems to work pretty well for the above params
