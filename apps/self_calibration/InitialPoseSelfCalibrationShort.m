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
 LoadConfigAndInit("IRMA8_diff_pulleys","IRMA8_diff_pulleys");
 ws_info = LoadWsInfo("8_cable_info");

% graphical visualization
cdpr_variables = UpdateIKZeroOrd([0;0;0],...
  [0;0;0],cdpr_parameters,cdpr_variables);
record.SetFrame(cdpr_variables,cdpr_parameters);

% JacobiansCheck(cdpr_parameters,cdpr_variables); % fix the tan jac and theta motor jac

% set parameters for optimal pose generation
axis_grid_points = [2 2 2;
                    3 2 3
                    3 3 3
                    4 3 4
                    4 4 4];
pose_bounds = [-1.2 1.2; -0.2 0.2; -1.6 0.8; 0 0; 0 0; 0 0];  %0 orient
% pose_bounds = [-1.4 1.4; -0.2 0.2; -1.6 1.1; -pi/24 pi/24;  -pi/6 pi/6; -pi/24 pi/24];

% assign disturb values
N = 50;
control_disturb.position_bias = 0;                                      %[m]
control_disturb.orientation_bias = 0;                                   %[rad]
control_disturb.position_noise = 0;                                     %[m]
control_disturb.orientation_noise = 0;                                  %[rad]
sensor_disturb.swivel_noise = deg2rad(3);                               %[rad]
sensor_disturb.length_noise = 0.04;                                     %[m]
sensor_disturb.AHRS_noise = deg2rad(3);                                 %[rad]
sensor_disturb.loadcell_noise = 10;                                      %[N]

%% Initial-Pose Self-Calibration simulation

% for meas_idx = 1:length(axis_grid_points)
for meas_idx = 1:3
    [Z_ideal,k] = GenerateConfigPosesBrutal(axis_grid_points(meas_idx,:),pose_bounds);
    Z_ideal=reshape(Z_ideal,[cdpr_parameters.pose_dim*k 1]);
    
    % %-------------SWIVEL AHRS sensor set----------------------------------
    % for disturb_idx = 1:N
    %     % IK simulation
    %     [X_real, delta_sigma_meas, delta_psi_meas, phi_meas, theta_meas] = ControlSimShortSwivelAHRS( ...
    %         cdpr_variables,cdpr_parameters,Z_ideal,k, ...
    %         control_disturb,sensor_disturb,1);
    % 
    %     % guess generation
    %     X_guess = Z_ideal;
    % 
    %     % % solve self-calibration problem
    %     % opts = optimoptions('lsqnonlin','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
    %     %     'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+5,'MaxIterations',1e+5);
    %     % tic
    %     % X_sol = lsqnonlin(@(X)CostFunSelfCalibShortSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
    %     %     k-1,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas),X_guess,[],[],[],[],[],[],[],...
    %     %     opts);
    %     % self_calib_comp_time=toc;
    % 
    %     % solve self-calibration problem
    %     opts = optimoptions('fmincon','SpecifyObjectiveGradient',true,'FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
    %         'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+5,'MaxIterations',1e+5);
    %     tic
    %     X_sol = fmincon(@(X)CostFunWeightSelfCalibShortSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
    %         k-1,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas,sensor_disturb),X_guess,[],[],[],[],[],[],[],opts);
    %     self_calib_comp_time=toc;
    % 
    %     InitialPositionErrorNorm(disturb_idx) = norm(X_real(1:3)-X_sol(1:3));
    %     cdpr_variables=UpdateIKZeroOrd(X_real(1:3),X_real(4:6),cdpr_parameters,cdpr_variables);
    %     angle_init_real = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
    %     cdpr_variables=UpdateIKZeroOrd(X_sol(1:3),X_sol(4:6),cdpr_parameters,cdpr_variables);
    %     angle_init_sol = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
    %     InitialOrientationError(disturb_idx) = rad2deg(abs(angle_init_sol-angle_init_real));
    % end
    % % store results
    % sc_output(1,meas_idx).InitialPositionErrorNorm = mean(InitialPositionErrorNorm);
    % sc_output(1,meas_idx).InitialOrientationError = mean(InitialOrientationError);
    % sc_output(1,meas_idx).InitPosErrNorm_min = min(InitialPositionErrorNorm);
    % sc_output(1,meas_idx).InitOrientErr_min = min(InitialOrientationError);
    % sc_output(1,meas_idx).InitPosErrNorm_max = max(InitialPositionErrorNorm);
    % sc_output(1,meas_idx).InitOrientErr_max = max(InitialOrientationError);
    % sc_output(1,meas_idx).NumberOfMeasures = k;
    % 
    % %-------------SWIVEL AHRS LENGTH sensor set------------------------
    % for disturb_idx = 1:N
    %     % IK simulation
    %     [X_real, delta_length_meas, delta_sigma_meas, delta_psi_meas, phi_meas, theta_meas] = ControlSimShortLengthSwivelAHRS( ...
    %         cdpr_variables,cdpr_parameters,Z_ideal,k, ...
    %         control_disturb,sensor_disturb,1);
    % 
    %     % guess generation
    %     X_guess = Z_ideal;
    % 
    %     % % solve self-calibration problem
    %     % opts = optimoptions('lsqnonlin','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
    %     %     'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+6,'MaxIterations',1e+6);
    %     % tic
    %     % X_sol = lsqnonlin(@(X)CostFunSelfCalibShortLengthSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
    %     %     k-1,delta_length_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas),X_guess,[],[],[],[],[],[],[],...
    %     %     opts);
    %     % self_calib_comp_time=toc;
    %     % solve self-calibration problem
    %     opts = optimoptions('fmincon','SpecifyObjectiveGradient',true,'FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
    %         'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+6,'MaxIterations',1e+6);
    %     tic
    %     X_sol = fmincon(@(X)CostFunWeightSelfCalibShortLengthSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
    %         k-1,delta_length_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas,sensor_disturb),X_guess,[],[],[],[],[],[],[],opts);
    %     self_calib_comp_time=toc;
    % 
    % 
    %     InitialPositionErrorNorm(disturb_idx) = norm(X_real(1:3)-X_sol(1:3));
    %     cdpr_variables=UpdateIKZeroOrd(X_real(1:3),X_real(4:6),cdpr_parameters,cdpr_variables);
    %     angle_init_real = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
    %     cdpr_variables=UpdateIKZeroOrd(X_sol(1:3),X_sol(4:6),cdpr_parameters,cdpr_variables);
    %     angle_init_sol = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
    %     InitialOrientationError(disturb_idx) = rad2deg(abs(angle_init_sol-angle_init_real));
    % end
    % % store results
    % sc_output(2,meas_idx).InitialPositionErrorNorm = mean(InitialPositionErrorNorm);
    % sc_output(2,meas_idx).InitialOrientationError = mean(InitialOrientationError);
    % sc_output(2,meas_idx).InitPosErrNorm_min = min(InitialPositionErrorNorm);
    % sc_output(2,meas_idx).InitOrientErr_min = min(InitialOrientationError);
    % sc_output(2,meas_idx).InitPosErrNorm_max = max(InitialPositionErrorNorm);
    % sc_output(2,meas_idx).InitOrientErr_max = max(InitialOrientationError);
    % sc_output(2,meas_idx).NumberOfMeasures = k;
    % 
    % %-------------SWIVEL AHRS LOADCELL sensor set--------------------------
    % for disturb_idx = 1:N
    %     % IK simulation
    %     [X_real, loadcell_meas, delta_sigma_meas, delta_psi_meas, phi_meas, theta_meas] = ControlSimShortLoadcellSwivelAHRS( ...
    %         cdpr_variables,cdpr_parameters,Z_ideal,k, ...
    %         control_disturb,sensor_disturb,1);
    % 
    %     % guess generation
    %     X_guess = Z_ideal;
    %     % % solve self-calibration problem
    %     % opts = optimoptions('lsqnonlin','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
    %     %     'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+5,'MaxIterations',1e+5);
    %     % tic
    %     % X_sol = lsqnonlin(@(X)CostFunSelfCalibShortLoadcellSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
    %     %     k-1,loadcell_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas),X_guess,[],[],[],[],[],[],[],...
    %     %     opts);
    %     % self_calib_comp_time=toc;
    % 
    %     % solve self-calibration problem
    %     opts = optimoptions('fmincon','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
    %         'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+5,'MaxIterations',1e+5);
    %     tic
    %     X_sol = fmincon(@(X)CostFunWeightSelfCalibShortLoadcellSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
    %         k-1,loadcell_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas,sensor_disturb),X_guess,[],[],[],[],[],[],[],opts);
    %     self_calib_comp_time=toc;
    % 
    % 
    %     InitialPositionErrorNorm(disturb_idx) = norm(X_real(1:3)-X_sol(1:3));
    %     cdpr_variables=UpdateIKZeroOrd(X_real(1:3),X_real(4:6),cdpr_parameters,cdpr_variables);
    %     angle_init_real = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
    %     cdpr_variables=UpdateIKZeroOrd(X_sol(1:3),X_sol(4:6),cdpr_parameters,cdpr_variables);
    %     angle_init_sol = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
    %     InitialOrientationError(disturb_idx) = rad2deg(abs(angle_init_sol-angle_init_real));
    % end
    % % store results
    % sc_output(3,meas_idx).InitialPositionErrorNorm = mean(InitialPositionErrorNorm);
    % sc_output(3,meas_idx).InitialOrientationError = mean(InitialOrientationError);
    % sc_output(3,meas_idx).InitPosErrNorm_min = min(InitialPositionErrorNorm);
    % sc_output(3,meas_idx).InitOrientErr_min = min(InitialOrientationError);
    % sc_output(3,meas_idx).InitPosErrNorm_max = max(InitialPositionErrorNorm);
    % sc_output(3,meas_idx).InitOrientErr_max = max(InitialOrientationError);
    % sc_output(3,meas_idx).NumberOfMeasures = k;

    %-------------SWIVEL AHRS LOADCELL LENGTH sensor set--------------------------
    for disturb_idx = 1:N
        % IK simulation
        [X_real, loadcell_meas, delta_length_meas, delta_sigma_meas, delta_psi_meas, phi_meas, theta_meas] = ControlSimShortLengthLoadcellSwivelAHRS( ...
            cdpr_variables,cdpr_parameters,Z_ideal,k, ...
            control_disturb,sensor_disturb,1);

        % guess generation
        X_guess = Z_ideal;
        % % solve self-calibration problem
        % opts = optimoptions('lsqnonlin','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
        %     'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+5,'MaxIterations',1e+5);
        % tic
        % X_sol = lsqnonlin(@(X)CostFunSelfCalibShortLoadcellSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
        %     k-1,loadcell_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas),X_guess,[],[],[],[],[],[],[],...
        %     opts);
        % self_calib_comp_time=toc;

        % solve self-calibration problem
        opts = optimoptions('fmincon','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
            'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+5,'MaxIterations',1e+5);
        tic
        X_sol = fmincon(@(X)CostFunWeightSelfCalibShortLengthLoadcellSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
            k-1,delta_length_meas,loadcell_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas,sensor_disturb),X_guess,[],[],[],[],[],[],[],opts);
        self_calib_comp_time=toc;


        InitialPositionErrorNorm(disturb_idx) = norm(X_real(1:3)-X_sol(1:3));
        cdpr_variables=UpdateIKZeroOrd(X_real(1:3),X_real(4:6),cdpr_parameters,cdpr_variables);
        angle_init_real = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
        cdpr_variables=UpdateIKZeroOrd(X_sol(1:3),X_sol(4:6),cdpr_parameters,cdpr_variables);
        angle_init_sol = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
        InitialOrientationError(disturb_idx) = rad2deg(abs(angle_init_sol-angle_init_real));
    end
    % store results
    sc_output(4,meas_idx).InitialPositionErrorNorm = mean(InitialPositionErrorNorm);
    sc_output(4,meas_idx).InitialOrientationError = mean(InitialOrientationError);
    sc_output(4,meas_idx).InitPosErrNorm_min = min(InitialPositionErrorNorm);
    sc_output(4,meas_idx).InitOrientErr_min = min(InitialOrientationError);
    sc_output(4,meas_idx).InitPosErrNorm_max = max(InitialPositionErrorNorm);
    sc_output(4,meas_idx).InitOrientErr_max = max(InitialOrientationError);
    sc_output(4,meas_idx).NumberOfMeasures = k;

    %-------------LOADCELL LENGTH sensor set--------------------------
    for disturb_idx = 1:N
        % IK simulation
        [X_real, loadcell_meas, delta_length_meas] = ControlSimShortLengthLoadcell( ...
            cdpr_variables,cdpr_parameters,Z_ideal,k, ...
            control_disturb,sensor_disturb,1);

        % guess generation
        X_guess = Z_ideal;
        % % solve self-calibration problem
        % opts = optimoptions('lsqnonlin','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
        %     'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+5,'MaxIterations',1e+5);
        % tic
        % X_sol = lsqnonlin(@(X)CostFunSelfCalibShortLoadcellSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
        %     k-1,loadcell_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas),X_guess,[],[],[],[],[],[],[],...
        %     opts);
        % self_calib_comp_time=toc;

        % solve self-calibration problem
        opts = optimoptions('fmincon','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
            'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+5,'MaxIterations',1e+5);
        tic
        X_sol = fmincon(@(X)CostFunWeightSelfCalibShortLengthLoadcell(cdpr_variables,cdpr_parameters,X,...
            k-1,delta_length_meas,loadcell_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas,sensor_disturb),X_guess,[],[],[],[],[],[],[],opts);
        self_calib_comp_time=toc;


        InitialPositionErrorNorm(disturb_idx) = norm(X_real(1:3)-X_sol(1:3));
        cdpr_variables=UpdateIKZeroOrd(X_real(1:3),X_real(4:6),cdpr_parameters,cdpr_variables);
        angle_init_real = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
        cdpr_variables=UpdateIKZeroOrd(X_sol(1:3),X_sol(4:6),cdpr_parameters,cdpr_variables);
        angle_init_sol = acos((cdpr_variables.platform.rot_mat(1,1)+cdpr_variables.platform.rot_mat(2,2)+cdpr_variables.platform.rot_mat(3,3)-1)/2);
        InitialOrientationError(disturb_idx) = rad2deg(abs(angle_init_sol-angle_init_real));
    end
    % store results
    sc_output(5,meas_idx).InitialPositionErrorNorm = mean(InitialPositionErrorNorm);
    sc_output(5,meas_idx).InitialOrientationError = mean(InitialOrientationError);
    sc_output(5,meas_idx).InitPosErrNorm_min = min(InitialPositionErrorNorm);
    sc_output(5,meas_idx).InitOrientErr_min = min(InitialOrientationError);
    sc_output(5,meas_idx).InitPosErrNorm_max = max(InitialPositionErrorNorm);
    sc_output(5,meas_idx).InitOrientErr_max = max(InitialOrientationError);
    sc_output(5,meas_idx).NumberOfMeasures = k;
end

% filename=strcat(folder,'/out_0orient',...
%     '_',num2str(max(sensor_disturb.length_noise)), ...
%     '_',num2str(rad2deg(max(sensor_disturb.swivel_noise))),...
%     '_',num2str(max(sensor_disturb.loadcell_noise)), ...
%     '_',num2str(rad2deg(max(sensor_disturb.AHRS_noise))),'.mat');
filename = strcat(folder,"/out_0orient","_combined_uniform_disturb45_");
save(filename,'sc_output')
%% Show results

PlotInitialPoseErrors(sc_output);