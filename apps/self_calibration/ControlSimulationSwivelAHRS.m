function X_real = ControlSimulationSwivelAHRS(cdpr_v,cdpr_p,Z_ideal,position_bias,orientation_bias,position_noise,orientation_noise)

% add control disturbances
pose_bias = repmat([position_bias*ones(3,1);...
    orientation_bias*ones(3,1)],k,1);
pose_noise = zeros(cdpr_parameters.pose_dim*k,1);
for i=1:k
    pose_noise(i*6-5:i*6) = [position_noise*(2*rand-1); ...
        position_noise*(2*rand-1);
        position_noise*(2*rand-1);
        orientation_noise*(2*rand-1);
        orientation_noise*(2*rand-1);
        orientation_noise*(2*rand-1)];
end
Z_ideal=reshape(Z_ideal,[cdpr_parameters.pose_dim*k 1]);
Z_real=Z_ideal+pose_bias+pose_noise;

% data acquisition simulation through IK
delta_sigma_meas = zeros(cdpr_parameters.n_cables,k);
delta_psi_meas = zeros(k,1);
phi_meas = zeros(k,1);
theta_meas = zeros(k,1);
sigma_0 = zeros(cdpr_parameters.n_cables,1);
psi_0 = 0;
for i = 1:k
    zeta_i = Z_real(6*i-5:6*i);
    cdpr_variables = UpdateIKZeroOrd(zeta_i(1:3),zeta_i(4:6),cdpr_p,cdpr_v);
    % delta swivel IK simulation
    for j = 1:cdpr_parameters.n_cables
        if i==1
            sigma_0(j) = cdpr_variables.cable(j).swivel_ang;
            delta_sigma_meas(j,i) = 0;
        else
            delta_sigma_meas(j,i) = cdpr_variables.cable(j).swivel_ang-...
                sigma_0(j);
        end
    end
    % delta yaw IK simulation
    if i==1
        psi_0 = cdpr_variables.platform.pose(6);
        delta_psi_meas(i) = 0;
    else
        delta_psi_meas(i) = cdpr_variables.platform.pose(6)-psi_0;
    end
    % roll and pitch IK simulation
    phi_meas(i) = cdpr_variables.platform.pose(4);
    theta_meas(i) = cdpr_variables.platform.pose(5);
end
X_real = [Z_real;sigma_0;psi_0];
end