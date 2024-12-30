clear
close all
clc

% transmission parameters (random values)
drum_length = 80;
flag_radius = 80;
motor_diameter = 40; 
q = 30;
s = 50;
motor_pitch = 4;
h = 40;

% motor angle discretization
N = 10000;
theta_m = linspace(0,drum_length*2*pi/motor_pitch,N);

norm_PT=zeros(size(theta_m));
[flag,error] = checkGradients(@(theta_)foo_calc_norm_PT(drum_length,...
    flag_radius,motor_diameter,q,s,motor_pitch,h,theta_),theta_m(250));

figure()
plot(norm_PT);
title('norm PT')

figure()
plot(P(1,:));
title('Px')

figure()
plot(beta);
title('beta')

figure()
plot(PA);
title('PA')