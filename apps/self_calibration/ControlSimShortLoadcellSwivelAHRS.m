function [X_real, loadcell_meas, delta_sigma_meas, delta_psi_meas, phi_meas, theta_meas] = ControlSimShortLoadcellSwivelAHRS(cdpr_v,cdpr_p,Z_ideal,k,control_disturb,loadcell_noise,swivel_noise,AHRS_noise)

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

% data acquisition simulation through IK
loadcell_opt_meas = zeros(cdpr_p.n_cables,k);
delta_sigma_opt_meas = zeros(cdpr_p.n_cables,k);
delta_psi_opt_meas = zeros(k,1);
phi_opt_meas = zeros(k,1);
theta_opt_meas = zeros(k,1);
sigma_0 = zeros(cdpr_p.n_cables,1);
psi_0 = 0;
for i = 1:k
    zeta_i = Z_real(6*i-5:6*i);
    cdpr_v = UpdateIKZeroOrd(zeta_i(1:3),zeta_i(4:6),cdpr_p,cdpr_v);
    cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
    J_d = cdpr_v.geometric_jacobian_l(:,1:6);
    J_c = cdpr_v.geometric_jacobian_l(:,7:8);
    w = cdpr_v.platform.ext_load;
    [tau_c, tau_d] = CalcTDBarycentric(cdpr_v,cdpr_p,J_d,J_c,w,[10 500]);
    % delta length and swivel IK simulation
    for j = 1:cdpr_p.n_cables
        if i==1
            sigma_0(j) = cdpr_v.cable(j).swivel_ang;
            delta_sigma_opt_meas(j,i) = 0;
        else
            delta_sigma_opt_meas(j,i) = cdpr_v.cable(j).swivel_ang-sigma_0(j);
        end
    end
    % delta yaw IK simulation and control law assignment
    if i==1
        psi_0 = cdpr_v.platform.pose(6);
        delta_psi_opt_meas(i) = 0;
        loadcell_opt_meas(:,i) = zeros(cdpr_p.n_cables,1);
    else
        delta_psi_opt_meas(i) = cdpr_v.platform.pose(6)-psi_0;
        loadcell_opt_meas(:,i) = [tau_d; tau_c];
    end
    % roll and pitch IK simulation
    phi_opt_meas(i) = cdpr_v.platform.pose(4);
    theta_opt_meas(i) = cdpr_v.platform.pose(5);
end
loadcell_opt_meas(:,1) = [];
delta_sigma_opt_meas(:,1) = [];
delta_psi_opt_meas(1) = [];
phi_opt_meas(1) = [];
theta_opt_meas(1) = [];
X_real = Z_real;

% add measurement disturbances
loadcell_meas = loadcell_noise*ones(size(loadcell_opt_meas))+loadcell_opt_meas;
delta_sigma_meas = swivel_noise*ones(size(delta_sigma_opt_meas))+delta_sigma_opt_meas;
delta_psi_meas = AHRS_noise*ones(size(delta_psi_opt_meas))+delta_psi_opt_meas;
phi_meas = AHRS_noise*(ones(size(phi_opt_meas)))+phi_opt_meas;
theta_meas = AHRS_noise*(ones(size(theta_opt_meas)))+theta_opt_meas;
end