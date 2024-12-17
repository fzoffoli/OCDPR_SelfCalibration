%This code simulates a feedforward cable length control for an OCDPR, aim 
% is to give an easy instrument that validates matrix and vector derivatives.

clear all
close all
clc

addpath('config')
addpath('data/workspace_files')
addpath('libs/cdpr_model')
addpath('libs/export_utilities')
addpath('libs/numeric')
addpath('libs/orientation_geometry')
addpath('libs/under_actuated')
addpath('libs/over_actuated')
folder = 'data';

%%% Parse config files, set initial pose and start record
[cdpr_parameters, cdpr_variables, cdpr_ws_data ,cdpr_outputs,record,utilities] =  LoadConfigAndInit("8_cable_IPAnema3_3","8_cable_IPAnema3_3");
cdpr_variables = UpdateIKZeroOrd([0;0;0],[0;0;0],cdpr_parameters,cdpr_variables);
record.SetFrame(cdpr_variables,cdpr_parameters);
ws_info = LoadWsInfo("8_cable_info");

%%% Trajectory parameters
dt = 0.01;                      % time step
Tmax = 5;                       % end time
pose_0 = cdpr_variables.platform.pose;
t=0:dt:Tmax;
pose_dt = [0.2;0;0;0;0;0];      % constant velocity motion law

%%% Hip force controlled cables are 7 and 8
l = zeros(cdpr_parameters.n_cables,length(t));
tau_c = zeros(cdpr_parameters.n_cables-cdpr_parameters.pose_dim,length(t));
tau = zeros(cdpr_parameters.n_cables,length(t));
tau_dt=zeros(cdpr_parameters.n_cables,length(t));
tau_dt_num=zeros(cdpr_parameters.n_cables,length(t));
for i = 1:length(t)
    pose = pose_0+t(i)*pose_dt;
    cdpr_variables = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_parameters,cdpr_variables);
    cdpr_variables = CalcExternalLoads(cdpr_variables,cdpr_parameters);
    [Jc,Jd,P] = PermJac_2dor(cdpr_variables.geometric_jacobian,7,8);
    
    l(:,i) = cdpr_variables.cable_vector;
    l_dt = cdpr_variables.geometric_jacobian'*pose_dt;
    tau_c(:,i)=(ws_info.tension_limits(2)+ws_info.tension_limits(1))/2*ones(cdpr_parameters.n_cables-cdpr_parameters.pose_dim,1); % constant tau_c equal to mean
    J_ort = [-(Jd\eye(6))*Jc; eye(cdpr_parameters.n_cables-cdpr_parameters.pose_dim)];
    
    %%% Analytic derivative d_tau/dt
    if i~=1
        delta_l = l(:,i)-l(:,i-1);
    else 
        delta_l = zeros(cdpr_parameters.n_cables,1);
    end
    [~,tauP_dl,tauN_dl,dJ_ort] = InputRatioIndex(cdpr_variables,cdpr_parameters,[7 8],Jd,Jc,tau_c,delta_l);
    tau_dl(:,:,i) = tauP_dl+tauN_dl;
    tau_dl_dl=tauP_dl*delta_l+dJ_ort*tau_c(:,i);
    if norm(tau_dl(:,:,i)*delta_l-tau_dl_dl)>1e-10 %%% Check of dJ_ort
        disp('fail!')
    end
    tau_dt(:,i)=tau_dl(:,:,i)*l_dt;
    

    %%% Numeric derivative d_tau/dt
    tau_P = [(Jd\eye(6)); zeros(cdpr_parameters.n_cables-cdpr_parameters.pose_dim,cdpr_parameters.pose_dim)]*cdpr_variables.platform.ext_load;
    tau(:,i) = tau_P+J_ort*tau_c(:,i);
    if i~=1
        tau_dt_num(:,i) = (tau(:,i)-tau(:,i-1))/dt;
    end
end


%% Derivative element plot
for i=1:cdpr_parameters.n_cables
    figure()
    subplot(2,1,1)
    plot(t(2:end),tau(i,2:end),'LineWidth',2);
    subplot(2,1,2)
    plot(t(2:end),tau_dt(i,2:end),'LineWidth',2);
    hold on
    plot(t(2:end),tau_dt_num(i,2:end),'LineWidth',2);
    legend('Analytic derivative','Numeric derivative');
end