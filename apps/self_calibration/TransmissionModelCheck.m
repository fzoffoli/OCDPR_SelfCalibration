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

[flag,error] = checkGradients(@(theta_m_)foo(cdpr_parameters.cable(1),theta_m_),0);
for i = 1:N
    l(i)=foo(cdpr_parameters.cable(1),theta_m(i));
    l_approx(i)=theta_m(i)*cdpr_parameters.cable(1).drum_diameter/2;
    l_approx(i)=l_approx(i)+l(1);
end

figure()
plot(rad2deg(theta_m),l,'LineWidth',1.5)
hold on
plot(rad2deg(theta_m),l_approx,'--','LineWidth',1.5)
xlabel('\theta_m [deg]')
ylabel('[m]')
legend('l_{real}','l_{approx}')
title('Cable length')

figure()
plot(rad2deg(theta_m),(l-l_approx)*1000,'LineWidth',1.5);
xlabel('\theta_m')
ylabel('[mm]')
title('Difference (l_{real}-l_{approx})')