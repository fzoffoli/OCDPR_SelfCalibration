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

% [cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
%   LoadConfigAndInit("Grab_prototype_44","RTR_Grab_prototype_44");
load trapz_set_points.mat
z = zeros(length(v),1);
data1 = [z z z v]; control1 = [z z z ones(length(v),1).*3];
data2 = [z z v z]; control2 = [z z ones(length(v),1).*3 z];
data3 = [z v z z]; control3 = [z ones(length(v),1).*3 z z];
data4 = [v z z z]; control4 = [ones(length(v),1).*3 z z z];
data = [data1;data2;data3;data4];
control = [control1;control2;control3;control4];
t = 0:0.001:(length(data)-1).*0.001;
motors_id = [0 1 2 3];
%data = t';
motors_id_str = num2str(motors_id);
name = "prova_over2";
filepath = strcat(name,".txt");
N = 4;
fid = fopen(filepath, 'w');
% 0 cable length
% 1 motor counts
% 2 motor speed 
% 3 motor torque
traj_type = 1;

relative = 0;
fprintf(fid, strcat('%d', repmat(' %d', 1, N), '\n'), [traj_type, motors_id]);
fprintf(fid, strcat('%f', repmat(' %d', 1, N), repmat(' %f', 1, N), '\n'), [t' control data]');
fclose(fid);
fprintf("File saved in %s\n", filepath)