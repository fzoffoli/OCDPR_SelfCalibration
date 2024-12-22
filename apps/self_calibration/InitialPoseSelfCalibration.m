%%% This App allows to calibrate the initial pose of an OCDPR using
%%% internal sensors. The robot parameters are loaded, then an ideal
%%% initial pose is provided and a suitable initial guess is given. Two
%%% approaches are used for self-calibration: with and without the cable
%%% lengths measures. Subsequently, a minimum number of EE congifurations 
%%% are computed to optimize the calibration results, using a calibration 
%%% index. Then, the optimization problems are build and solved using the 
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
cdpr_variables = UpdateIKZeroOrd([0.5;0;0],...
  [0;0;0],cdpr_parameters,cdpr_variables);
record.SetFrame(cdpr_variables,cdpr_parameters);

% JacobiansCheck(cdpr_parameters,cdpr_variables); % fix the tan_jac bug

% set parameters for optimal pose generation
k = 20;
pose_bounds = [-1.5 1.5; -0.3 0.3; -1.5 1; -pi/12 pi/12; -pi/6 pi/6; -pi/12 pi/12];
Z_bounds = repmat(pose_bounds,k,2);
method = OptimalConfigurationMethod.MIN_CONDITION_NUM;

% generate N poses for optimal calibration
Z_ideal = ga(@(Z)FitnessFunSwivelAHRS(cdpr_variables,cdpr_parameters,Z,k,method),...
    k*cdpr_parameters.pose_dim,[],[],[],[],Z_bounds(:,1),Z_bounds(:,2),...
    @(Z)NonlconWorkspaceBelonging(cdpr_variables,cdpr_parameters,Z,k,ws_info));

% adding distrubances and noise to obtain realistic measures and guesses
X_ideal = [Z_ideal; zeros(k*(3+cdpr_parameters.pose_dim),1)];

% solve self-calibration problem
opts = optimoptions();
X_sol = lsqnonlin(@(X)CostFunSelfCalibrationSwivelAHRS(cdpr_v,cdpr_p,X,...
    k,delta_sigma,roll,pitch,delta_yaw),X_ideal,[],[],[],[],[],[],[],...
    opts);

% show results
