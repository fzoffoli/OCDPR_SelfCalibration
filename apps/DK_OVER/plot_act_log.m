clc; clear; close all;

%% Parse file

addpath('../../config')
addpath('../../data/workspace_files')
addpath('../../libs/cdpr_model')
addpath('../../libs/export_utilities')
addpath('../../libs/numeric')
addpath('../../libs/orientation_geometry')
addpath('../../libs/under_actuated')
addpath('../../libs/over_actuated')
addpath('../../libs/prototype_log_parser')
addpath('../../libs/prototype_log_parser/msgs')
[cdpr_parameters, cdpr_variables, ~ ,cdpr_outputs,record,utilities] = ...
LoadConfigAndInit("Grab_prototype_44_planar","Grab_prototype_44_planar");
filepath = 'calib12.log';
data = parseCableRobotLogFile(filepath);
plot_flags = [1,0,0,0,0];

%% Pack according to actuator ID
[sorted_id, sorting_idx] = sort(data.actuator_status.values.id);

actuators_id = unique(sorted_id);
num_actuators = length(actuators_id);
torques_mat = zeros(num_actuators, length(sorted_id)/num_actuators);
motor_pos_mat = torques_mat;
motor_vel_mat = torques_mat;
pulley_enc_mat = torques_mat;
pulley_angle_mat = torques_mat;
sorted_ts = torques_mat;
for i = 1:num_actuators
    idx = sorting_idx(sorted_id == actuators_id(i));
    torques_mat(i, :) = data.actuator_status.values.motor_torque(idx);
    motor_pos_mat(i, :) = data.actuator_status.values.motor_position(idx);
    motor_vel_mat(i, :) = data.actuator_status.values.motor_speed(idx);
    pulley_enc_mat(i, :) = data.actuator_status.values.aux_position(idx);
    pulley_angle_mat(i, :) = data.actuator_status.values.pulley_angle(idx);
    sorted_ts(i, :) = data.actuator_status.timestamp(idx);
end

%% Plot torques
if plot_flags(1)
%     figure('units','normalized','outerposition',[0 0 1 1])
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), torques_mat(i,:))
        grid on
        title(sprintf('Actuator #%d torques', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot positions
if plot_flags(2)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), motor_pos_mat(i,:))
        grid on
        title(sprintf('Actuator #%d position', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot velocities
if plot_flags(2)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), motor_vel_mat(i,:))
        grid on
        title(sprintf('Actuator #%d velocity', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot pulley encoder values
if plot_flags(4)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), pulley_enc_mat(i,:))
        grid on
        title(sprintf('Actuator #%d encoder values', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot pulley angles
if plot_flags(5)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), pulley_angle_mat(i,:))
        grid on
        title(sprintf('Actuator #%d pulley angles', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end

%cdpr_variables.cable_vector = [0.698;0.711;0.466;0.424];
cdpr_variables.cable_vector = [0.401;0.414;0.757;0.748];
m = [0.321;0.631;1.040];
m_pl = 2.430;

[pose0,fval] = fsolve(@(p) DK_fun(p,cdpr_variables.cable_vector,cdpr_parameters,cdpr_variables),[0.0;0;0.0;0;0;0],utilities.fsolve_options);
cdpr_variables = UpdateIKZeroOrd(pose0(1:3),pose0(4:6),cdpr_parameters,cdpr_variables);
record.SetFrame(cdpr_variables,cdpr_parameters);

t = sorted_ts(1,:)';
tension_list = torques_mat';
tension = [];
app = [];
for i=1:length(t)-1
    if (t(i+1)-t(i))<0.1
        app = [app;tension_list(i+1,:)];
    else
        tension = [tension; mean(app(:,1)) mean(app(:,2)) mean(app(:,3)) mean(app(:,4))];
        app = [];
    end
end
tension = [tension; mean(app(:,1)) mean(app(:,2)) mean(app(:,3)) mean(app(:,4))];

ID_mat = [];
v_ext = [];
g = -9.81;
J = [cdpr_variables.geometric_jacobian(1,:); cdpr_variables.geometric_jacobian(3,:); cdpr_variables.geometric_jacobian(5,:)];

for i=1:length(tension)
   
   sub_id = J*[eye(4) diag(tension(i,:))];
   ID_mat = [ID_mat; sub_id];
   switch i
       case 1
           v_ext = [v_ext;0;g*m_pl;0];
      
       case 2
           v_ext = [v_ext;0;g*(m(1)+m_pl);0];
     
       case 3
           v_ext = [v_ext;0;g*(m(2)+m_pl);0];
     
       case 4
           v_ext = [v_ext;0;g*(m(1)+m(2)+m_pl);0];
    
%        case 5
%            v_ext = [v_ext;0;g*(m(1)+m(2)+m_pl);0];
%      
%        case 6
%            v_ext = [v_ext;0;g*(m(1)+m(3)+m_pl);0];
%       
%        case 7
%            v_ext = [v_ext;0;g*(m(3)+m(2)+m_pl);0];
%         
%        case 8
%            v_ext = [v_ext;0;g*(m(1)+m(2)+m(3)+m_pl);0];
           
   end
   
end

linsolve(ID_mat'*ID_mat,ID_mat'*v_ext)

