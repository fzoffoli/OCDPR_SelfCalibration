clear
close all
clc

addpath('../../config')
addpath('../../data')
addpath('../../github_repo')
addpath('../../libs')
addpath('../../apps/self_calibration');
addpath('../../apps/workspace_computation');
addpath('../../data/workspace_files')
addpath('../../libs/cdpr_model')
addpath('../../libs/export_utilities')
addpath('../../libs/numeric')
addpath('../../libs/orientation_geometry')
addpath('../../libs/under_actuated')
addpath('../../libs/over_actuated')
addpath('../../libs/logger_reader')
addpath('../../libs/prototype_log_parser')
folder = '../../data';


% load robot parameters
[cdpr_parameters, cdpr_variables, cdpr_ws_data ,cdpr_outputs,record,utilities] = ...
 LoadConfigAndInit("IRMA8_platform_prot","IRMA8_platform_prot");
 ws_info = LoadWsInfo("8_cable_info");

% motor angle discretization
N = 10000;
theta_m = linspace(0,cdpr_parameters.cable(1).drum_length*2*pi/cdpr_parameters.cable(1).drum_pitch,N);
% l = linspace(0,5,N);

% [flag,error] = checkGradients(@(theta_m_)foo(cdpr_parameters.cable(1),theta_m_),0);
for i = 1:N
    l(i)=foo(cdpr_parameters.cable(1),theta_m(i));
    % l_approx(i)=theta_m(i)*cdpr_parameters.cable(1).drum_diameter/2;
    % l_approx(i)=l_approx(i)+l(1);

    % [Delta(i),P(i),D(i),R(i),Delta_0(i)] = CalcMotorAngleBrutal(cdpr_parameters.cable(1).flag_radius, ...
    %     cdpr_parameters.cable(1).drum_diameter,cdpr_parameters.cable(1).q,cdpr_parameters.cable(1).s, ...
    %     cdpr_parameters.cable(1).drum_pitch,cdpr_parameters.cable(1).h,cdpr_parameters.cable(1).C,l(i));
    
    [theta_motor(i), J_theta_m(i)] = CalcMotorAngleBrutal(cdpr_parameters.cable(1).flag_radius, ...
        cdpr_parameters.cable(1).drum_diameter,cdpr_parameters.cable(1).q,cdpr_parameters.cable(1).s, ...
        cdpr_parameters.cable(1).drum_pitch,cdpr_parameters.cable(1).h,cdpr_parameters.cable(1).C,l(i));
end

J_theta_m_numeric = diff(theta_m)./diff(l);
% figure()
% plot(rad2deg(theta_m),l,'LineWidth',1.5)
% hold on
% plot(rad2deg(theta_m),l_approx,'--','LineWidth',1.5)
% grid on
% xlabel('\theta_m [deg]')
% ylabel('[m]')
% legend('l_{real}','l_{approx}')
% title('Cable length')
% 
% figure()
% plot(rad2deg(theta_m),(l-l_approx)*1000,'LineWidth',1.5);
% grid on
% xlabel('\theta_m')
% ylabel('[mm]')
% title('Difference (l_{real}-l_{approx})')

% figure()
% plot(Delta)
% title('\Delta')
% 
% figure()
% plot(P)
% title('P')
% 
% figure()
% plot(D)
% title('D')
% 
% figure()
% plot(R)
% title('R')
% 
% figure()
% plot(Delta_0)
% title('\Delta_0')

figure()
plot(theta_motor)
plot(l,rad2deg(theta_m),'LineWidth',1.5)
hold on
plot(l,rad2deg(theta_motor),'--','LineWidth',1.5)
grid on
xlabel('l [m]')
ylabel('[deg]')
legend('\theta_m initial','\theta_m inverse')
title('\theta_m')

figure()
plot(l(2:end),J_theta_m(2:end),'LineWidth',1.5)
hold on
plot(l(2:end),J_theta_m_numeric,'--','LineWidth',1.5)
grid on
xlabel('l [m]')
ylabel('[deg/m]')
legend('J_{\theta_m} analytic','J_{\theta_m} numeric')
title('J_{\theta_m}')