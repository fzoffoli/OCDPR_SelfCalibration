%%% This App allows to calibrate the initial pose of an OCDPR using
%%% internal sensors. The robot parameters are loaded, then an ideal
%%% initial pose is provided and a suitable initial guess is given. Two
%%% approaches are used for self-calibration: with and without the cable
%%% lengths measures. Subsequently, a minimum number of EE congifurations 
%%% are computed to optimize the calibration results, using a calibration 
%%% index. The optimization problems are build and solved using the 
%%% optimal poses. Finally, the two calibration results are shown and compared.
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

% graphical visualization
cdpr_variables = UpdateIKZeroOrd([0;0;0],...
  [0;0;0],cdpr_parameters,cdpr_variables);
record.SetFrame(cdpr_variables,cdpr_parameters);

% JacobiansCheck(cdpr_parameters,cdpr_variables); % fix the tan_jac bug

% set parameters for optimal pose generation
for k=10:10:60
% k = 10;
pose_bounds = [-1.4 1.4; -0.2 0.2; -1.6 1.1; 0 0; 0 0; 0 0];
Z_bounds = repmat(pose_bounds,k,2);
method = OptimalConfigurationMethod.MIN_CONDITION_NUM;

% generate k poses for optimal calibration
Z_not_ideal = [linspace(pose_bounds(1,1),pose_bounds(1,2),k);
                linspace(pose_bounds(2,1),pose_bounds(2,2),k);
                linspace(pose_bounds(3,1),pose_bounds(3,2),k);
                zeros(1,k);
                zeros(1,k);
                zeros(1,k)];
Z_not_ideal=reshape(Z_not_ideal,[k*cdpr_parameters.pose_dim 1]);
opts_ga = optimoptions('ga','UseParallel',true);
tic
Z_ideal = ga(@(Z)FitnessFunSwivelAHRS(cdpr_variables,cdpr_parameters,Z,k,method),...
    k*cdpr_parameters.pose_dim,[],[],[],[],Z_bounds(:,1),Z_bounds(:,2),...
    @(Z)NonlconWorkspaceBelonging(cdpr_variables,cdpr_parameters,Z,k,ws_info),opts_ga);
opt_pose_comp_time = toc;
save(strcat('calib_pose_0orient_',num2str(k)),"Z_ideal",...
    'cdpr_parameters','cdpr_variables','k',"opt_pose_comp_time");

%% adding distrubances in the simulation to obtain realistic measures and guesses
%----------I control disturbances (bias and noise)---------------
delta_max = 5;
position_control_bias = linspace(0.0,0.040,delta_max);                %[m]
orientation_control_bias = linspace(0,deg2rad(3),delta_max);    %[deg]
position_control_noise = 0.002;               %[m]
orientation_control_noise = deg2rad(1);   %[rad]
for delta=1:delta_max
pose_bias = repmat([position_control_bias(delta)*ones(3,1);...
    orientation_control_bias(delta)*ones(3,1)],k,1);
pose_noise = zeros(cdpr_parameters.pose_dim*k,1);
for i=1:k
    pose_noise(i*6-5:i*6) = [position_control_noise*(2*rand-1); ...
        position_control_noise*(2*rand-1);
        position_control_noise*(2*rand-1);
        orientation_control_noise*(2*rand-1);
        orientation_control_noise*(2*rand-1);
        orientation_control_noise*(2*rand-1)];
end

Z_ideal=reshape(Z_ideal,[cdpr_parameters.pose_dim*k 1]);
Z_real=Z_ideal+pose_bias+pose_noise;
%----------------------------------------------------------------
%---------II state-estimation disturbances (bias and noise)------
delta_sigma_meas = zeros(cdpr_parameters.n_cables,k);
delta_psi_meas = zeros(k,1);
phi_meas = zeros(k,1);
theta_meas = zeros(k,1);
sigma_0 = zeros(cdpr_parameters.n_cables,1);
psi_0 = 0;
for i = 1:k
    zeta_i = Z_real(6*i-5:6*i);
    cdpr_variables = UpdateIKZeroOrd(zeta_i(1:3),zeta_i(4:6),cdpr_parameters,cdpr_variables);
    % delta swivel IK simulation
    for j = 1:cdpr_parameters.n_cables
        if i==1
            sigma_0(j) = cdpr_variables.cable(j).swivel_ang;
            delta_sigma_meas(j,i) = 0;
        else
            delta_sigma_meas(j,i) = cdpr_variables.cable(j).swivel_ang-...
                sigma_0(j);
        end
    end
    % delta yaw IK simulation
    if i==1
       psi_0 = cdpr_variables.platform.pose(6);
       delta_psi_meas(i) = 0; 
    else
       delta_psi_meas(i) = cdpr_variables.platform.pose(6)-psi_0;
    end
    % roll and pitch IK simulation
    phi_meas(i) = cdpr_variables.platform.pose(4);
    theta_meas(i) = cdpr_variables.platform.pose(5);
end
X_real = [Z_real;sigma_0;psi_0];
X_guess = [Z_ideal;zeros(cdpr_parameters.n_cables+1,1)];


% solve self-calibration problem
opts = optimoptions('lsqnonlin','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8,'UseParallel',true);
tic
X_sol = lsqnonlin(@(X)CostFunSelfCalibrationSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
    k,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas),X_guess,[],[],[],[],[],[],[],...
    opts);
self_calib_comp_time=toc;

% store and show results
output.sc_comp_time = self_calib_comp_time;
output.X_real = X_real;
output.X_sol = X_sol;
output.InitialPositionErrorNorm = norm(X_real(1:3)-X_sol(1:3));

cdpr_variables=UpdateIKZeroOrd(X_real(1:3),X_real(4:6),cdpr_parameters,cdpr_variables);
angle_init_real = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
cdpr_variables=UpdateIKZeroOrd(X_sol(1:3),X_sol(4:6),cdpr_parameters,cdpr_variables);
angle_init_sol = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
output.InitialOrientationError = rad2deg(abs(angle_init_sol-angle_init_real));

filename=strcat(folder,'/out_',num2str(k), ...
    '_',strrep(num2str(position_control_bias(delta)),'.',''), ...
    '_',strrep(num2str(position_control_noise),'.',''),...
    '_',strrep(num2str(rad2deg(orientation_control_bias(delta))),'.',''), ...
    '_',strrep(num2str(rad2deg(orientation_control_noise)),'.',''),'.mat');
save(filename,"Z_ideal",'cdpr_parameters','cdpr_variables','k','output')

disp(output);
end
end