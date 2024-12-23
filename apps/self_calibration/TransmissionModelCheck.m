clear
close all
clc

% transmission parameters (random values)
l_d = 80;
R = 80;
d_m = 40; 
q = 30;
s = 50;
rho = 4;
h = 40;

% motor angle discretization
N = 10000;
theta_m = linspace(0,l_d*2*pi/rho,N);

norm_PT=zeros(size(theta_m));
[flag,error] = checkGradients(@(theta_)foo_calc_norm_PT(l_d,R,d_m,q,s,rho,h,theta_),theta_m(250));

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