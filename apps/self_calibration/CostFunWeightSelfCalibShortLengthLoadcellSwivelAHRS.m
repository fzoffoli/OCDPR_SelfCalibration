function f = CostFunWeightSelfCalibShortLengthLoadcellSwivelAHRS(cdpr_v,cdpr_p,X,k,delta_length,tau,delta_sigma,roll,pitch,delta_yaw,sensor_disturb)
% This is the cost function for a self-calibration optimization problem
% where the initial pose of a CDPR has to be estimated using loadcells 
% measures, swivel angle variation measures and euler angles from an AHRS.
% Read Zoffoli2025 CableCon for the complete formulation.

% set constants for the weighing matrix
force_max = sensor_disturb.loadcell_noise;
length_max = sensor_disturb.length_noise;
sigma_max = sensor_disturb.swivel_noise;
epsilon_max = sensor_disturb.AHRS_noise;

% extract variables and parameters 
zeta_0 = X(1:cdpr_p.pose_dim);
Z = X(cdpr_p.pose_dim+1:cdpr_p.pose_dim*(k+1));
cdpr_v = UpdateIKZeroOrd(zeta_0(1:3),zeta_0(4:6),cdpr_p,cdpr_v);
length_0 = cdpr_v.cable_vector;
sigma_0 = zeros(cdpr_p.n_cables,1);
for j_=1:cdpr_p.n_cables
    sigma_0(j_) = cdpr_v.cable(j_).swivel_ang;
end
psi_0 = zeta_0(end);

% fill the residual vector
n = cdpr_p.n_cables;
f_tau = zeros(k*6,1);
f_length = zeros(k*n,1);
f_sigma = zeros(k*n,1);
f_epsilon = zeros(k*3,1);
for i=1:k
    zeta_k = Z(i*cdpr_p.pose_dim-(cdpr_p.pose_dim-1):i*cdpr_p.pose_dim);
    cdpr_v = UpdateIKZeroOrd(zeta_k(1:3),zeta_k(4:6),cdpr_p,cdpr_v);
    cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
    length_model = zeros(n,1);
    sigma_model=zeros(n,1);
    for j_=1:n
        length_model(j_) = cdpr_v.cable(j_).complete_length;
        sigma_model(j_) = cdpr_v.cable(j_).swivel_ang;
    end
    % [tau_c, tau_d] = CalcTDBarycentric(cdpr_v,cdpr_p,[10 500]);
    % f_tau(i*n-(n-1):i*n) = [tau_d;tau_c]-tau(:,i); 
    % wrench = cdpr_v.geometric_jacobian_l*tau(:,i)-cdpr_v.platform.ext_load; 
    % f_tau(i*6-(6-1):i*6) = [wrench(1:3)./force_max; wrench(4:6)./moment_max];
    tau_zeta = CalcTDClosedForm(cdpr_v,cdpr_p,[10 500]);
    f_tau(i*n-(n-1):i*n) = (tau_zeta-tau(:,i)); 
    f_length(i*n-(n-1):i*n) = length_model-length_0-delta_length(:,i); 
    f_sigma(i*n-(n-1):i*n) = (sigma_model-sigma_0-delta_sigma(:,i));
    f_epsilon(i*3-2:i*3) = (zeta_k(4:6)-[roll(i);pitch(i);delta_yaw(i)]-[0;0;psi_0]);
end
F = [f_tau; f_length; f_sigma; f_epsilon];
% W = diag([repmat([1/(force_max^2); 1/(force_max^2); 1/(force_max^2); ...
%                     1/(moment_max^2); 1/(moment_max^2); 1/(moment_max^2)],[k 1]); ...
%             ones(size(f_sigma))./(sigma_max^2); ...
%             ones(size(f_epsilon))./(epsilon_max^2)]);
W = diag([ones(size(f_tau))./(force_max^2); ...
            ones(size(f_length))./(length_max^2); ...
            ones(size(f_sigma))./(sigma_max^2); ...
            ones(size(f_epsilon))./(epsilon_max^2)]);
f = 0.5*F'*W*F;
end