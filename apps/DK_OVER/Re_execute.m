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
  LoadConfigAndInit("Grab_prototype_44_planar","HomingTest44");

% cdpr_variables = UpdateIKZeroOrd([0;0;0],[0;0;0],cdpr_parameters,cdpr_variables);
% record.SetFrame(cdpr_variables,cdpr_parameters);

pose0 = [-0.11;0;-0.06;0;0;0];

dt = 0.001;

tau0 = [40;37;45;44];
T_tens = 3;
tau_com = 35;

cdpr_variables = UpdateIKZeroOrd(pose0(1:3),pose0(4:6),cdpr_parameters,cdpr_variables);
J = [cdpr_variables.geometric_jacobian(1,:);cdpr_variables.geometric_jacobian(3,:);cdpr_variables.geometric_jacobian(5,:)];
J_null = [-linsolve(J(:,1:3),J(:,4));1];
ind = find(abs(J_null)==max(abs(J_null)));
index = ind(1);
Jd = [J(:,1:index-1) J(:,index+1:end)];
   Jc = J(:,index);
   T = linsolve(Jd,[0;cdpr_parameters.platform.mass*cdpr_parameters.platform.gravity_acceleration(3);0]-Jc*tau_com);
  tau(1:index-1,1) = T(1:length(1:index-1));
  tau(index,1) = tau_com;
  tau(index+1:4,1) = T(length(1:index-1)+1:end);

%index = 4;
time_tens = 0:dt:T_tens;
L = length(time_tens);
tens_cmd = tau0(index):(tau(index)-tau0(index))/(L-1):tau(index);
if isempty(tens_cmd)
   tens_cmd = tau_com.*ones(1,L); 
end
dlu = 0:0.005/(L-2)*2:0.005;
dld = 0.005:-0.005/(L-2)*2:0;
dl = [dlu 0.005 dld];
for i=1:L
   cmd_tens(i,:) = cdpr_variables.cable_vector';
   cmd_tens(i,index) = tens_cmd(i);
   cmd_tens(i,1:index-1) = cmd_tens(i,1:index-1)+dl(i);
   cmd_tens(i,index+1:end) = cmd_tens(i,index+1:end)+dl(i);
   ctrl_tens(i,:) = zeros(1,4);
   ctrl_tens(i,index) = 2;
end

p_start = [pose0(1);pose0(3);pose0(5)];
T_start = 5;
time_pos = T_tens+dt:dt:T_tens+T_start;
L = length(time_pos);
pose_start = pose0;
for i=1:6
    if (norm(pose_start(i,:)-pose0(i,:))>0.0001)
        pos_cmd(i,:) = pose0(i,:):(pose_start(i,:)-pose0(i,:))/(L-1):pose_start(i,:);
    else
        pos_cmd(i,:) = pose0(i,:).*ones(1,L);
    end
end

for i=1:L
   cdpr_variables = UpdateIKZeroOrd(pos_cmd(1:3,i),pos_cmd(4:6,i),cdpr_parameters,cdpr_variables);
   J = [cdpr_variables.geometric_jacobian(1,:);cdpr_variables.geometric_jacobian(3,:);cdpr_variables.geometric_jacobian(5,:)];
   J_null = [-linsolve(J(:,1:3),J(:,4));1];
  ind = find(abs(J_null)==max(abs(J_null)));
index = ind(1);
Jd = [J(:,1:index-1) J(:,index+1:end)];
   Jc = J(:,index);
   T = linsolve(Jd,[0;cdpr_parameters.platform.mass*cdpr_parameters.platform.gravity_acceleration(3);0]-Jc*tau_com);
  tau(1:index-1,1) = T(1:length(1:index-1));
  tau(index,1) = tau_com;
  tau(index+1:4,1) = T(length(1:index-1)+1:end);

%index = 4;
   
   
   
   cmd_pos(i,:) = cdpr_variables.cable_vector';
   cmd_pos(i,index) = tau(index);
   ctrl_pos(i,:) = zeros(1,4);
   ctrl_pos(i,index) = 2;
end

T_wait = 3;
time_wait = T_tens+T_start+dt:dt:T_tens+T_start+T_wait;
L = length(time_wait);

for i=1:L
   cmd_wait(i,:) = cmd_pos(end,:);
   ctrl_wait(i,:) = ctrl_pos(end,:);
end


r = 0.05;
p0 = p_start;
c = [p0(1)+r;p0(2)];
v_m = 0.05;
T = 2*pi*r/v_m;

i=0;
for t=0:dt:T
    i=i+1;
    time_exp(i,1) = T_tens+T_start+T_wait+dt+t;
    p(1:2,i) = [c(1)-r*cos(2*pi*t/T);c(2)-r*sin(2*pi*t/T)];
    p(3,i) = p0(3);
end
L = length(time_exp);

for i=1:L
   pose = [p(1,i);0;p(2,i);0;p(3,i);0];
   cdpr_variables = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_parameters,cdpr_variables);
   J = [cdpr_variables.geometric_jacobian(1,:);cdpr_variables.geometric_jacobian(3,:);cdpr_variables.geometric_jacobian(5,:)];
   J_null = [-linsolve(J(:,1:3),J(:,4));1];
   ind = find(abs(J_null)==max(abs(J_null)));
index = ind(1);
   Jd = [J(:,1:index-1) J(:,index+1:end)];
   Jc = J(:,index);
   T = linsolve(Jd,[0;cdpr_parameters.platform.mass*cdpr_parameters.platform.gravity_acceleration(3);0]-Jc*tau_com);
  tau(1:index-1,1) = T(1:length(1:index-1));
  tau(index,1) = tau_com;
  tau(index+1:4,1) = T(length(1:index-1)+1:end);

%index = 4;
  
  tau_test(:,i) = tau;
   
   cmd_exp(i,:) = cdpr_variables.cable_vector';
   cmd_exp(i,index) = tau(index);
   ctrl_exp(i,:) = zeros(1,4);
   ctrl_exp(i,index) = 2;
    
end

time = [time_tens';time_pos';time_wait';time_exp];
ctrl = [ctrl_tens;ctrl_pos;ctrl_wait;ctrl_exp];
cmd = [cmd_tens;cmd_pos;cmd_wait;cmd_exp];

motors_id = [0 1 2 3];
%data = t';
motors_id_str = num2str(motors_id);
name = "prova_over";
filepath = strcat(name,".txt");
N = 4;
fid = fopen(filepath, 'w');
% 0 cable length
% 1 motor counts
% 2 motor speed 
% 3 motor torque
traj_type = 0;


fprintf(fid, strcat('%d', repmat(' %d', 1, N), '\n'), [traj_type, motors_id]);
fprintf(fid, strcat('%f', repmat(' %d', 1, N), repmat(' %f', 1, N), '\n'), [time ctrl cmd]');
fclose(fid);
fprintf("File saved in %s\n", filepath)
