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
addpath('../../github_repo')
folder = '../../data';

[cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Grab_prototype_44_planar","HomingTest44");

% cdpr_variables = UpdateIKZeroOrd([0;0;0],[0;0;0],cdpr_parameters,cdpr_variables);
% record.SetFrame(cdpr_variables,cdpr_parameters);

cdpr_variables.cable_vector = [0.729;0.683;0.418;0.473];

[pose0,fval] = fsolve(@(p) DK_fun(p,cdpr_variables.cable_vector,cdpr_parameters,cdpr_variables),[0.0;0;0.0;0;0;0],utilities.fsolve_options);
cdpr_variables = UpdateIKZeroOrd(pose0(1:3),pose0(4:6),cdpr_parameters,cdpr_variables);
record.SetFrame(cdpr_variables,cdpr_parameters);
j_struct.init_pose = pose0';
json.startup
json.write(j_struct, 'homing_results.json')
fprintf('Results dumped in %s\n', strcat(pwd, '/results.json'))

dt = 0.001;

tau0 = [19.4;9.4;15.9;25.8];
T_tens = 3;
tau_com = 45;

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
if (isempty(tens_cmd))
    tens_cmd = tau_com.*ones(L,1);
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

%%%%% INPORT PROCESS DATA, called data.
load('data1.mat')
     p_start = [data(1,2);data(1,3);0];


%%%%% QUI METTERE IL P0
%p_start = [-0.11;-0.06;0];
T_start = 5;
time_pos = T_tens+dt:dt:T_tens+T_start;
L = length(time_pos);
pose_start = [p_start(1);0;p_start(2);0;p_start(3);0];
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

time = [time_tens';time_pos';time_wait'];
ctrl = [ctrl_tens;ctrl_pos;ctrl_wait];
cmd = [cmd_tens;cmd_pos;cmd_wait];



mat0 =  [time ctrl cmd]';

motors_id = [0 1 2 3];
%data = t';
motors_id_str = num2str(motors_id);
name = "prova_over_0";
filepath = strcat(name,".txt");
N = 4;
fid = fopen(filepath, 'w');
% 0 cable length
% 1 motor counts
% 2 motor speed 
% 3 motor torque
traj_type = 0;

fprintf(fid, strcat('%d', repmat(' %d', 1, N), '\n'), [traj_type, motors_id]);
fprintf(fid, strcat('%f', repmat(' %d', 1, N), repmat(' %f', 1, N), '\n'), mat0);
fclose(fid);
fprintf("File saved in %s\n", filepath)
figure;
hold on
for jj=1:5
    clear time_exp
    clear ctrl_exp
    clear cmd_exp
    clear time
    clear ctrl
    clear cmd
    clear mat0
    data_name = strcat('data',num2str(jj),'.mat');
    load(data_name);
    scatter(data(:,2),data(:,3));
    T = data(end,1);
    i=0;
    for t=0:dt:T
        i=i+1;
        time_exp(i,1) = t;
        p(1:2,i) = data(i,2:3)';
        p(3,i) = 0;
    end
    L = length(time_exp);
    for i=1:L
        pose = [p(1,i);0;p(2,i);0;p(3,i);0];
        ppose(:,i) = pose;
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
        tau_test(:,i) = tau;
   
        cmd_exp(i,:) = cdpr_variables.cable_vector';
        cmd_exp(i,index) = tau(index);
        ctrl_exp(i,:) = zeros(1,4);
        ctrl_exp(i,index) = 2; 
    end
    time = [time_exp];
    ctrl = [ctrl_exp];
    cmd = [cmd_exp];
    mat0 =  [time ctrl cmd]';
    
    motors_id = [0 1 2 3];
    motors_id_str = num2str(motors_id);
    name = strcat("prova_over_",num2str(jj));
    filepath = strcat(name,".txt");
    N = 4;
    fid = fopen(filepath, 'w');
% 0 cable length
% 1 motor counts
% 2 motor speed 
% 3 motor torque
    traj_type = 0;

    fprintf(fid, strcat('%d', repmat(' %d', 1, N), '\n'), [traj_type, motors_id]);
    fprintf(fid, strcat('%f', repmat(' %d', 1, N), repmat(' %f', 1, N), '\n'), mat0);
    fclose(fid);
    fprintf("File saved in %s\n", filepath)
    
end
hold off
%%%%%%% DA QUA
%r = 0.05;
% p0 = p_start;
% %c = [p0(1)+r;p0(2)];
% %v_m = 0.06;
% %T = 2*pi*r/v_m;
% %%%% T DA modificare come da  file di input
% T = data(end,1);
% 
% i=0;
% for t=dt:dt:T
%     i=i+1;
%     time_exp(i,1) = T_tens+T_start+T_wait+t;
%     %%%%%% QUA DA METTERE QUELLO DEL FILE DI INPUT
%     %p(1:2,i) = [c(1)-r*cos(2*pi*t/T);c(2)-r*sin(2*pi*t/T)];
%     p(1:2,i) = data(i,2:3)';
%     p(3,i) = p0(3);
% end
% L = length(time_exp);
% 
% for i=1:L
%    pose = [p(1,i);0;p(2,i);0;p(3,i);0];
%    ppose(:,i) = pose;
%    cdpr_variables = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_parameters,cdpr_variables);
%    J = [cdpr_variables.geometric_jacobian(1,:);cdpr_variables.geometric_jacobian(3,:);cdpr_variables.geometric_jacobian(5,:)];
%    J_null = [-linsolve(J(:,1:3),J(:,4));1];
%    ind = find(abs(J_null)==max(abs(J_null)));
% index = ind(1);
%    Jd = [J(:,1:index-1) J(:,index+1:end)];
%    Jc = J(:,index);
%    T = linsolve(Jd,[0;cdpr_parameters.platform.mass*cdpr_parameters.platform.gravity_acceleration(3);0]-Jc*tau_com);
%   tau(1:index-1,1) = T(1:length(1:index-1));
%   tau(index,1) = tau_com;
%   tau(index+1:4,1) = T(length(1:index-1)+1:end);
% 
%  %index = 4;
%   
%   tau_test(:,i) = tau;
%    
%    cmd_exp(i,:) = cdpr_variables.cable_vector';
%    cmd_exp(i,index) = tau(index);
%    ctrl_exp(i,:) = zeros(1,4);
%    ctrl_exp(i,index) = 2;
%     
% end
% 
% time = [time_tens';time_pos';time_wait';time_exp];
% ctrl = [ctrl_tens;ctrl_pos;ctrl_wait;ctrl_exp];
% cmd = [cmd_tens;cmd_pos;cmd_wait;cmd_exp];
% 
% 
% 
% mat =  [time ctrl cmd]';
% 
% T_end = mat(1,end);
% t_div =round(T_end/4*1000)/1000;
% idx_incr = t_div*1000-1;
% idx_start = 1;
% idx_stop = idx_incr;
% 
% 
% for jj=1:4
% motors_id = [0 1 2 3];
% %data = t';
% motors_id_str = num2str(motors_id);
% name = strcat("prova_over_",num2str(jj));
% filepath = strcat(name,".txt");
% N = 4;
% fid = fopen(filepath, 'w');
% % 0 cable length
% % 1 motor counts
% % 2 motor speed 
% % 3 motor torque
% traj_type = 0;
% 
% fprintf(fid, strcat('%d', repmat(' %d', 1, N), '\n'), [traj_type, motors_id]);
% mat2 = mat(:,idx_start:idx_stop);
% mat2(1,:) = mat2(1,:)-mat2(1,1);
% fprintf(fid, strcat('%f', repmat(' %d', 1, N), repmat(' %f', 1, N), '\n'), mat2);
% fclose(fid);
% fprintf("File saved in %s\n", filepath)
% idx_start = idx_start+idx_incr;
% idx_stop = idx_stop+idx_incr;
% end
