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

% JacobiansCheck(cdpr_parameters,cdpr_variables); % fix the tan jac and theta motor jac

% set parameters for optimal pose generation
% k_set=10:10:30;
flag_cable_lengths = 1;
k_set=51;
pose_bounds = [-1.4 1.4; -0.2 0.2; -1.6 1.1; 0 0; 0 0; 0 0];  %0 orient
% pose_bounds = [-1.4 1.4; -0.2 0.2; -1.6 1.1; -pi/24 pi/24;  -pi/6 pi/6; -pi/24 pi/24];

%% Optimal Configuration generation
for i=1:length(k_set)
    k = k_set(i);
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
    
    % solve the optimal configuration problem
    opts_ga = optimoptions('ga','UseParallel',true);
    tic
    if ~flag_cable_lengths
        Z_ideal = ga(@(Z)FitnessFunSwivelAHRS(cdpr_variables,cdpr_parameters,Z,k,method),...
            k*cdpr_parameters.pose_dim,[],[],[],[],Z_bounds(:,1),Z_bounds(:,2),...
            @(Z)NonlconWorkspaceBelonging(cdpr_variables,cdpr_parameters,Z,k,ws_info),opts_ga);
    else
        % Z_ideal = ga(@(Z)FitnessFunSwivelAHRSMotor(cdpr_variables,cdpr_parameters,Z,k,method),...
        %     k*cdpr_parameters.pose_dim,[],[],[],[],Z_bounds(:,1),Z_bounds(:,2),...
        %     @(Z)NonlconWorkspaceBelonging(cdpr_variables,cdpr_parameters,Z,k,ws_info),opts_ga);
        Z_ideal = ga(@(Z)FitnessFunLengthSwivelAHRS(cdpr_variables,cdpr_parameters,Z,k,method),...
            k*cdpr_parameters.pose_dim,[],[],[],[],Z_bounds(:,1),Z_bounds(:,2),...
            @(Z)NonlconWorkspaceBelonging(cdpr_variables,cdpr_parameters,Z,k,ws_info),opts_ga);
        opt_pose_comp_time = toc;
    end
    % store data
    save(strcat('calib_pose_0orient_',num2str(k)),"Z_ideal",...
        'cdpr_parameters','cdpr_variables','k',"opt_pose_comp_time");
end
%% Initial-Pose Self-Calibration simulation
for meas_idx = 1:length(k_set)
    % load measure set
    k = k_set(meas_idx);
    load(strcat('calib_pose_0orient_',num2str(k),'.mat'))
    
    % assign disturb values
    disturb_mesh = 5;
    position_control_bias = linspace(0.0,0.040,disturb_mesh);          %[m]
    orientation_control_bias = linspace(0,deg2rad(3),disturb_mesh);    %[rad]
    position_control_noise = 0.002;           %[m]
    orientation_control_noise = deg2rad(1);   %[rad]
    for disturb_idx=1:length(position_control_bias)

        % control disturb simulation
        if ~flag_cable_lengths
            Z_ideal=reshape(Z_ideal,[cdpr_parameters.pose_dim*k 1]);
            [X_real, delta_sigma_meas, delta_psi_meas, phi_meas, theta_meas] = ControlSimulationSwivelAHRS( ...
                cdpr_variables,cdpr_parameters,Z_ideal,k, ...
                position_control_bias(disturb_idx), ...
                orientation_control_bias(disturb_idx), ...
                position_control_noise,orientation_control_noise);

            % guess generation
            cdpr_variables = UpdateIKZeroOrd(Z_ideal(1:3),Z_ideal(4:6),cdpr_parameters,cdpr_variables);
            sigma_0_guess = zeros(cdpr_parameters.n_cables,1);
            for j = 1:cdpr_parameters.n_cables
                sigma_0_guess(j) = cdpr_variables.cable(j).swivel_ang;
            end
            X_guess = [Z_ideal;sigma_0_guess;Z_ideal(6)];
            X_lb = [repmat([-2.4;-0.3;-1.7;-pi;-pi;-pi],k,1); -pi*ones(...
                cdpr_parameters.n_cables+1,1)];
            X_ub = [repmat([2.4;0.3;1.7;pi;pi;pi],k,1); pi*ones(...
                cdpr_parameters.n_cables+1,1)];

            % solve self-calibration problem
            opts = optimoptions('lsqnonlin','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
                'StepTolerance',1e-8,'UseParallel',true);
            tic
            X_sol = lsqnonlin(@(X)CostFunSelfCalibrationSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
                k,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas),X_guess,X_lb,X_ub,[],[],[],[],[],...
                opts);
            self_calib_comp_time=toc;
        else
            Z_ideal=reshape(Z_ideal,[cdpr_parameters.pose_dim*k 1]);
            [X_real, delta_length_meas, delta_sigma_meas, delta_psi_meas, phi_meas, theta_meas] = ControlSimLengthSwivelAHRS( ...
                cdpr_variables,cdpr_parameters,Z_ideal,k, ...
                position_control_bias(disturb_idx), ...
                orientation_control_bias(disturb_idx), ...
                position_control_noise,orientation_control_noise);

            % guess generation
            cdpr_variables = UpdateIKZeroOrd(Z_ideal(1:3),Z_ideal(4:6),cdpr_parameters,cdpr_variables);
            length_0_guess = zeros(cdpr_parameters.n_cables,1);
            sigma_0_guess = zeros(cdpr_parameters.n_cables,1);
            for j = 1:cdpr_parameters.n_cables
                length_0_guess(j) = cdpr_variables.cable(j).complete_length;
                sigma_0_guess(j) = cdpr_variables.cable(j).swivel_ang;
            end
            X_guess = [Z_ideal;length_0_guess;sigma_0_guess;Z_ideal(6)];
            X_lb = [repmat([-2.4;-0.3;-1.7;-pi;-pi;-pi],k,1); -pi*ones(...
                2*cdpr_parameters.n_cables+1,1)];
            X_ub = [repmat([2.4;0.3;1.7;pi;pi;pi],k,1); pi*ones(...
                2*cdpr_parameters.n_cables+1,1)];

            % solve self-calibration problem
            opts = optimoptions('lsqnonlin','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
                'StepTolerance',1e-8,'UseParallel',true);
            tic
            X_sol = lsqnonlin(@(X)CostFunSelfCalibrationLengthSwivelAHRS(cdpr_variables,cdpr_parameters,X,...
                k,delta_length_meas,delta_sigma_meas,phi_meas,theta_meas,delta_psi_meas),X_guess,X_lb,X_ub,[],[],[],[],[],...
                opts);
            self_calib_comp_time=toc;
        end

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

        filename=strcat(folder,'/out_0orient_',num2str(k), ...
            '_',strrep(num2str(position_control_bias(disturb_idx)),'.',''), ...
            '_',strrep(num2str(position_control_noise),'.',''),...
            '_',strrep(num2str(rad2deg(orientation_control_bias(disturb_idx))),'.',''), ...
            '_',strrep(num2str(rad2deg(orientation_control_noise)),'.',''),'.mat');
        % save(filename,"Z_ideal",'cdpr_parameters','cdpr_variables','k','output')

        disp(output);
    end
end