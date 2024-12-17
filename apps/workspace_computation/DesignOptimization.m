clear 
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

%%% choose the desired robot
 % [cdpr_parameters, cdpr_variables, cdpr_ws_data ,cdpr_outputs,record,utilities] = ...
 % LoadConfigAndInit("8_cable_HRPCable","8_cable_HRPCable");
  [cdpr_parameters, cdpr_variables, cdpr_ws_data ,cdpr_outputs,record,utilities] = ...
 LoadConfigAndInit("8_cable_IPAnema3_3","8_cable_IPAnema3_3");
 %  [cdpr_parameters, cdpr_variables, cdpr_ws_data ,cdpr_outputs,record,utilities] = ...
 % LoadConfigAndInit("Laser_engraver_planar","Laser_engraver_planar");
% cdpr_variables = UpdateIKZeroOrd([-2.54314; 0.864; 1.178],... %5;-3.5;0.4    0; 0; 0.2
%   [0;0;0],cdpr_parameters,cdpr_variables);
cdpr_variables = UpdateIKZeroOrd([0; 0; 0],... %5;-3.5;0.4    0; 0; 0.2
  [0;0;0],cdpr_parameters,cdpr_variables);
record.SetFrame(cdpr_variables,cdpr_parameters);%
 ws_info = LoadWsInfo("8_cable_info");

% %%% Geometric scaling procedure
% if strcmp('8_cable_IPAnema3_3',record.figure_handle.FileName)
%     k=2.013659; % geometric similarity constant IPANEMA-HRP for the same installation ws
%     for i=1:cdpr_parameters.n_cables
%         cdpr_parameters.cable(i).pos_PA_loc=(1/k)*cdpr_parameters.cable(i).pos_PA_loc;
%         cdpr_parameters.cable(i).pos_OD_glob=(1/k)*cdpr_parameters.cable(i).pos_OD_glob;
%     end
%     cdpr_variables = UpdateIKZeroOrd([0; 0; 0],... %5;-3.5;0.4    0; 0; 0.2
%         [0;0;0],cdpr_parameters,cdpr_variables);
%     record.SetFrame(cdpr_variables,cdpr_parameters);%
%     ws_info = LoadWsInfo("8_cable_info");
% end
    
%%% Setting nl programming solvers options
opt_gradient = utilities.brutal_fmincon_options;
opt_gradient.FiniteDifferenceStepSize = 0.01;
opt_gradient.Display = 'iter-detailed';
opt_gradient.OptimalityTolerance = 1e-6;
opt_gradient.StepTolerance = 1e-10;
% options = optimoptions("ga",'FitnessLimit',-0.5,'MaxGenerations',10*3*cdpr_parameters.n_cables,'PopulationSize',10,'FunctionTolerance',1e-2,'UseParallel',true,'HybridFcn',{@fmincon,opt_gradient});
% options = optimoptions("ga",'FitnessLimit',-0.5,'MaxGenerations',10*3*cdpr_parameters.n_cables,'PopulationSize',10,'FunctionTolerance',1e-2,'UseParallel',true);
for i = 1:8
    attach_p(i,:)=cdpr_parameters.cable(i).pos_PA_loc';
end
attach_P=reshape(attach_p,[24,1]);
% lb=attach_P-0.5*abs(attach_P);
% ub=attach_P+0.5*abs(attach_P);
lb=[0.03; 0.03; -0.10; -0.10; 0.050; 0.050; -0.13; -0.13; 0.32; -0.97; -0.97; 0.32;...
    0.37; -1.12; -1.12; 0.37; -0.40; -0.40; -0.40; -0.40; 0.13; 0.13; 0.13; 0.13];
ub=[0.09; 0.09; -0.04; -0.04; 0.14; 0.14; -0.04; -0.04; 0.97; -0.33; -0.33; 0.97;...
    1.12; -0.37; -0.37; 1.12; -0.13; -0.13; -0.13; -0.13; 0.40; 0.40; 0.40; 0.40];

break_cnt = 1;
%% first cc optimization
[cc_idx_old,cdpr_outputs] = OptCableComb(cdpr_parameters,cdpr_variables,cdpr_outputs,utilities,folder,record,ws_info);
while(1)

    % geometric optimization
    fun = @(X)-FitnessFunPlatformGeom(X,cc_idx_old,cdpr_parameters,cdpr_variables,cdpr_outputs,utilities,folder,record,ws_info);
    nvars = 3*cdpr_parameters.n_cables;
    for i = 1:8
        attach_p(i,:)=cdpr_parameters.cable(i).pos_PA_loc';
    end
    attach_P=reshape(attach_p,[24,1]);
    % X_sol = ga(fun,nvars,[],[],[],[],lb,ub,[],options);
    X_sol = fmincon(fun,attach_P,[],[],[],[],lb,ub,[],opt_gradient);
    
    % intermediate saving
    save(strcat('ipanema_internal_opt3_',num2str(break_cnt)),'X_sol','cc_idx_old');

    % update geometric parameters
    X_sol=reshape(X_sol,[8 3]);
    for i = 1:cdpr_parameters.n_cables
        cdpr_parameters.cable(i).pos_PA_loc=X_sol(i,:)';
    end

    % check cc optimization
    [cc_idx_new,cdpr_outputs] = OptCableComb(cdpr_parameters,cdpr_variables,cdpr_outputs,utilities,folder,record,ws_info);
    
    if cc_idx_new==cc_idx_old
        sol.geom=X_sol;
        sol.cc=cc_idx_old;
        break
    elseif break_cnt >10
        disp('algorithm not converged');
        break
    else
        cc_idx_old=cc_idx_new;
    end
    break_cnt = break_cnt+1;
end

save('ipanema_grad_opt_mmt_3.mat');
%% Compile to show data
close all

%%% choose the correct title
% sim_title="8_cable_HRPCable";
sim_title="8_cable_IPAnema3_3";
% sim_title="Laser_engraver_planar";
config_name=sim_title;

%BOUNDARY WS
ws_info = LoadWsInfo("8_cable_info");
PlotWorkspaceBound(sim_title,config_name,cdpr_parameters,cdpr_variables,ws_info,cc_idx_old,cdpr_outputs)