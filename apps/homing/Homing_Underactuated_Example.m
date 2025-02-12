clear all
close all
clc
addpath('../../config')
addpath('../../data/workspace_files')
addpath('../../libs/cdpr_model')
addpath('../../libs/export_utilities')
addpath('../../libs/numeric')
addpath('../../libs/orientation_geometry')
addpath('../../libs/under_actuated')
addpath('../../libs/over_actuated')
folder = '../../data';

[cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Grab_prototype_44","HomingTest44");

homing_info = LoadHomingInfo("Homing_info");
% % Generation of the "roughly" estimated home pose

start = SimulateGoToStartProcedureErroneous...
    (cdpr_parameters, cdpr_variables,ws_parameters,record,utilities,homing_info.tension_limits);
home = SimulateGoToStartProcedureIdeal...
    (cdpr_parameters, cdpr_variables,start.pose,record,utilities,homing_info.tension_limits);
[delta_l,delta_sw] = SimulateDataAcquisition(cdpr_parameters, cdpr_variables, homing_info.steps4cable,home,record,utilities,homing_info.tension_limits);
init_guess = GenerateInitialGuess(cdpr_parameters,cdpr_variables,delta_l,start,record,utilities);

% Contruct external patameter and initial guess
init_guess = [start.l;start.sw;init_guess];
[sol,resnorm,residual,exitflag,output] = lsqnonlin(@(v)HomingOptimizationFunction...
  (cdpr_parameters,record,delta_l,delta_sw,v),init_guess ,[],[],utilities.lsqnonlin_options_grad);
l0 = sol(1:cdpr_parameters.n_cables,1);
s0 = sol(cdpr_parameters.n_cables+1:2*cdpr_parameters.n_cables,1);
[pose0,resnormp,residualp,exitflagp,outputp] = lsqnonlin(@(v)FunDkGsSwL...
  (cdpr_parameters,l0,s0,v,record),start.pose ,[],[],utilities.lsqnonlin_options_grad);




