clear all
close all
clc

% w1 = [1630;2649;3676;4697;5743;6787]/1000*9.81;
% meas1 = [275;520;768;1014;1265;1518];

% w2 = [1631;2649;3676;4698;5741;6787]/1000*9.81;
% meas2 = [492;743;996;1250;1510;1768];

% w0 = [1631;2649;3676;4698;5741;6787]/1000*9.81;
% meas0 = [277;518;761;1003;1250;1500];

w = [1631;2649;3676;4698;5741;6787]/1000*9.81;
meas = [580;830;1075;1326;1579;1836];

A = [ones(6,1),meas];
data = linsolve(A'*A,A'*w)

meas_tot = 0:1:3300;
tau = data(1) +data(2).*meas_tot;
hold on
plot(meas_tot,tau);
plot(meas,w);
hold off

%(0.0449-0.043714531181106)/0.043714531181106