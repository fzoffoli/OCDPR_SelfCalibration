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

norm_PT=zeros(size(theta_m));
[flag,error] = checkGradients(@(theta_)foo_calc_norm_PT(cdpr_parameters.cable(1).drum_length,...
    cdpr_parameters.cable(1).flag_radius,cdpr_parameters.cable(1).drum_diameter, ...
    cdpr_parameters.cable(1).q,cdpr_parameters.cable(1).s,cdpr_parameters.cable(1).drum_pitch, ...
    cdpr_parameters.cable(1).h,theta_),theta_m(250));
for i = 1:N
    norm_PT(i)=foo_calc_norm_PT(cdpr_parameters.cable(1).drum_length,...
    cdpr_parameters.cable(1).flag_radius,cdpr_parameters.cable(1).drum_diameter, ...
    cdpr_parameters.cable(1).q,cdpr_parameters.cable(1).s,cdpr_parameters.cable(1).drum_pitch, ...
    cdpr_parameters.cable(1).h,theta_m(i));
end
figure()
plot(norm_PT);
title('norm PT')

% figure()
% plot(P(1,:));
% title('Px')
% 
% figure()
% plot(beta);
% title('beta')
% 
% figure()
% plot(PA);
% title('PA')