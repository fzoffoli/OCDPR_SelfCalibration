function [X_real, loadcell_meas, delta_length_meas] = ControlSimShortLengthLoadcell(cdpr_v,cdpr_p,Z_ideal,k,control_disturb,sensor_disturb,scale_factor)

% add control disturbances
pose_bias = repmat([control_disturb.position_bias*ones(3,1);...
    control_disturb.orientation_bias*ones(3,1)],k,1);
pose_noise = zeros(cdpr_p.pose_dim*k,1);
for i=1:k
    pose_noise(i*6-5:i*6) = [control_disturb.position_noise*(2*rand-1); ...
        control_disturb.position_noise*(2*rand-1);
        control_disturb.position_noise*(2*rand-1);
        control_disturb.orientation_noise*(2*rand-1);
        control_disturb.orientation_noise*(2*rand-1);
        control_disturb.orientation_noise*(2*rand-1)];
end
Z_real=Z_ideal+pose_bias+pose_noise;

% compute sensor disturbances
loadcell_noise = scale_factor*sensor_disturb.loadcell_noise;
length_noise = scale_factor*sensor_disturb.length_noise;

% data acquisition simulation through IK
loadcell_opt_meas = zeros(cdpr_p.n_cables,k);
delta_length_opt_meas = zeros(cdpr_p.n_cables,k);
loadcell_meas = zeros(cdpr_p.n_cables,k);
delta_length_meas = zeros(cdpr_p.n_cables,k);
length_0 = zeros(cdpr_p.n_cables,1);
for i = 1:k
    zeta_i = Z_real(6*i-5:6*i);
    cdpr_v = UpdateIKZeroOrd(zeta_i(1:3),zeta_i(4:6),cdpr_p,cdpr_v);
    % [tau_c, tau_d] = CalcTDBarycentric(cdpr_v,cdpr_p,[10 500]);
    tau = CalcTDClosedForm(cdpr_v,cdpr_p,[10 500]);
    % delta length and swivel IK simulation
    for j = 1:cdpr_p.n_cables
        if i==1
            length_0(j) = cdpr_v.cable(j).complete_length;
            delta_length_opt_meas(j,i) = 0;
            delta_length_meas(j,i) = delta_length_opt_meas(j,i)+(2*rand-1)*length_noise;
        else
            delta_length_opt_meas(j,i) = cdpr_v.cable(j).complete_length-length_0(j);
            delta_length_meas(j,i) = delta_length_opt_meas(j,i)+(2*rand-1)*length_noise;
        end
    end
    % delta yaw IK simulation and control law assignment
    if i==1
        loadcell_opt_meas(:,i) = zeros(cdpr_p.n_cables,1);
        loadcell_meas(:,i) = loadcell_opt_meas(:,i)+(2*rand(cdpr_p.n_cables,1) ...
            -ones(cdpr_p.n_cables,1))*loadcell_noise;
    else
        % loadcell_opt_meas(:,i) = [tau_d; tau_c];
        loadcell_opt_meas(:,i) = tau;
        loadcell_meas(:,i) = loadcell_opt_meas(:,i)+(2*rand(cdpr_p.n_cables,1) ...
            -ones(cdpr_p.n_cables,1))*loadcell_noise;
    end
end

loadcell_meas(:,1) = [];
delta_length_meas(:,1) = [];

X_real = Z_real;
end